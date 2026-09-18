//
// Created by biao on 24-10-6.
//

#ifndef OBSERVATIONBUFFER_H
#define OBSERVATIONBUFFER_H

#include <torch/torch.h>
#include <vector>

class ObservationBuffer
{
public:
    ObservationBuffer(int num_envs, int num_obs, int include_history_steps);

    ~ObservationBuffer() = default;

    void reset(const std::vector<int>& reset_index, const torch::Tensor& new_obs);

    void clear();

    void insert(const torch::Tensor& new_obs);

    [[nodiscard]] torch::Tensor getObsVec(const std::vector<int>& obs_ids) const;

    /** Return all buffered samples in Isaac Lab's per-observation-term layout.
     *
     * For term dimensions [3, 3, 12], the result is
     * [term0(t-N)..term0(t), term1(t-N)..term1(t), term2(t-N)..term2(t)].
     */
    [[nodiscard]] torch::Tensor getTermMajorObsVec(const std::vector<int>& term_dims) const;

private:
    int num_envs_;
    int num_obs_;
    int include_history_steps_;
    int num_obs_total_;
    torch::Tensor obs_buffer_;
};


#endif //OBSERVATIONBUFFER_H
