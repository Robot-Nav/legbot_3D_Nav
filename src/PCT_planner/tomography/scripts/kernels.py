import string
import cupy as cp


def utils_point(resolution, n_row, n_col):
    util_preamble = string.Template(
        '''
        __device__ int getIndexLine(float16 x, float16 center)
        {
            int i = round((x - center) / ${resolution});
            return i;
        }

        __device__ int getIndexMap_1d(float16 x, float16 y, float16 cx, float16 cy)
        {
            // Return 1D index of a point (x, y) in a layer
            int idx_x = getIndexLine(x, cx) + ${n_row} / 2;
            int idx_y = getIndexLine(y, cy) + ${n_col} / 2;

            // Check if the index is inside the map
            if (idx_x < 0 || idx_x >= ${n_row} || idx_y < 0 || idx_y >= ${n_col})
            {
                return -1;
            }
            return ${n_col} * idx_x + idx_y;
        }

        __device__ int getIndexBlock_1d(int idx, int layer_n)
        {
            // Return 1D index of a point (x, y) in multi-layer map block
            return (int)${layer_size} * layer_n + idx;
        }

        __device__ static float atomicMaxFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fmaxf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }

        __device__ static float atomicMinFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fminf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }
        '''
    ).substitute(
        resolution=resolution,
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def utils_map(n_row, n_col):
    util_preamble=string.Template(
        '''
        __device__ int getIdxRelative(int idx, int dx, int dy) 
        {
            // Return 1D index of the relative point (x+dx, y+dy) in multi-layer map block
            int idx_2d = idx % (int)${layer_size};
            int idx_x = idx_2d / ${n_col};
            int idx_y = idx_2d % ${n_col};
            int idx_rx = idx_x + dx;
            int idx_ry = idx_y + dy;

            if ( idx_rx < 0 || idx_rx > (${n_row} - 1) ) 
                return -1;
            if ( idx_ry < 0 || idx_ry > (${n_col} - 1) )
                return -1;

            return ${n_col} * dx + dy + idx;
        }
        '''
    ).substitute(
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def tomographyKernel(resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
    tomography_kernel = cp.ElementwiseKernel(
        in_params='raw U points, raw U center',
        out_params='raw U layers_g, raw U layers_c',
        preamble=utils_point(resolution, n_row, n_col),
        operation=string.Template(
            '''
            U px = points[i * 3];
            U py = points[i * 3 + 1];
            U pz = points[i * 3 + 2];

            int idx = getIndexMap_1d(px, py, center[0], center[1]);
            if ( idx < 0 ) 
                return; 
            for ( int s_idx = 0; s_idx < ${n_slice}; s_idx ++ )
            {
                U slice = ${slice_h0} + s_idx * ${slice_dh};
                if ( pz <= slice )
                    atomicMaxFloat(&layers_g[getIndexBlock_1d(idx, s_idx)], pz);
                else
                    atomicMinFloat(&layers_c[getIndexBlock_1d(idx, s_idx)], pz);
            }
            '''
        ).substitute(
            n_slice=n_slice,
            slice_h0=slice_h0,
            slice_dh=slice_dh
        ),
        name='tomography_kernel'
    )
                            
    return tomography_kernel


def travKernel(
    n_row, n_col, half_kernel_size,
    interval_min, interval_free, step_cross, step_stand, standable_th, cost_barrier
    ):
    trav_kernel = cp.ElementwiseKernel(
        in_params='raw U interval, raw U grad_mag_sq, raw U grad_mag_max',
        out_params='raw U trav_cost',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            if ( interval[i] < ${interval_min} )
            {
                trav_cost[i] = ${cost_barrier};
                return;
            }
            else
                trav_cost[i] += max(0.0, 20 * (${interval_free} - interval[i]));
            if ( grad_mag_sq[i] <= ${step_stand_sq} )
            {
                trav_cost[i] += 15 * grad_mag_sq[i] / ${step_stand_sq};
                return;
            }
            else 
            {
                if ( grad_mag_max[i] <= ${step_cross_sq} )
                {
                    int standable_grids = 0;
                    for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
                    {
                        for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                        {
                            int idx = getIdxRelative(i, dx, dy);
                            if ( idx < 0 )
                                continue;
                            if ( grad_mag_sq[idx] < ${step_stand_sq} )
                                standable_grids += 1;
                        }
                    }
                    if ( standable_grids < ${standable_th} )
                    {
                        trav_cost[i] = ${cost_barrier};
                        return;
                    }
                    else
                        trav_cost[i] += 20 * grad_mag_max[i] / ${step_cross_sq};
                }
                else
                {
                    trav_cost[i] = ${cost_barrier};
                    return;
                }
            } 
            '''
        ).substitute(
            half_kernel_size=half_kernel_size,
            interval_min=interval_min,
            interval_free=interval_free,
            step_cross_sq=step_cross ** 2,
            step_stand_sq=step_stand ** 2,
            standable_th=standable_th,
            cost_barrier=cost_barrier
        ),
        name='trav_kernel'
    )
                            
    return trav_kernel


def inflationKernel(n_row, n_col, half_kernel_size):
    inflation_kernel = cp.ElementwiseKernel(
        in_params='raw U trav_cost, raw U score_table',
        out_params='raw U inflated_cost',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            int counter = 0;
            for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
            {
                for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                {
                    int idx = getIdxRelative(i, dx, dy);
                    if ( idx >= 0 )
                        inflated_cost[i] = max(inflated_cost[i], trav_cost[idx] * score_table[counter]);
                    counter += 1;
                }
            }
            '''
        ).substitute(
            half_kernel_size=half_kernel_size
        ),
        name='inflation_kernel'
    )
                            
    return inflation_kernel


def distanceTransformKernel(n_row, n_col, half_kernel_size, cost_barrier):
    dt_kernel = cp.ElementwiseKernel(
        in_params='raw U trav_cost',
        out_params='raw U dist_to_barrier',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            if (trav_cost[i] >= ${cost_barrier} - 0.001f) {
                dist_to_barrier[i] = 0.0f;
                return;
            }
            float min_d = 1e6f;
            for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
            {
                for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                {
                    int idx = getIdxRelative(i, dx, dy);
                    if ( idx >= 0 && trav_cost[idx] >= ${cost_barrier} - 0.001f) {
                        float d = sqrtf((float)(dx*dx + dy*dy));
                        if (d < min_d) min_d = d;
                    }
                }
            }
            dist_to_barrier[i] = min_d;  
            '''
        ).substitute(
            half_kernel_size=half_kernel_size,
            cost_barrier=cost_barrier
        ),
        name='distance_transform_kernel'
    )
    return dt_kernel


def adjustCostByDistanceKernel(n_row, n_col,safe_margin, cost_barrier,resolution):
    """Increase trav_cost for cells close to barrier: smaller distance -> higher cost"""
    adjust_kernel = cp.ElementwiseKernel(
        in_params='raw U trav_cost, raw U dist_to_barrier',
        out_params='raw U trav_cost_out',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            if (trav_cost[i] >= ${cost_barrier} - 0.001f) {
                trav_cost_out[i] = trav_cost[i];      // already barrier, keep unchanged
                return;
            }

            float dist = dist_to_barrier[i];
            float max_influence_dist = 1.5*${safe_margin}/${resolution};   //1.5 times the safe margin
            float penalty = trav_cost_out[i] * (dist / max_influence_dist);
            trav_cost_out[i]=max(0.001f,trav_cost_out[i]-penalty);
            '''
        ).substitute(
            cost_barrier=cost_barrier,
            safe_margin=safe_margin,
            resolution=resolution
        ),
        name='adjust_cost_by_distance_kernel'
    )
    return adjust_kernel