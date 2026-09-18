//
// Created by biao on 24-10-6.
//

#include "ObservationBuffer.h"

ObservationBuffer::ObservationBuffer(int num_envs,
                                     const int num_obs,
                                     const int include_history_steps)
    : num_envs_(num_envs),
      num_obs_(num_obs),
      include_history_steps_(include_history_steps) {
    num_obs_total_ = num_obs_ * include_history_steps_;
    obs_buffer_ = torch::zeros({num_envs_, num_obs_total_}, dtype(torch::kFloat32));
}

void ObservationBuffer::reset(const std::vector<int> &reset_index, const torch::Tensor &new_obs) {
    std::vector<torch::indexing::TensorIndex> indices;
    for (int index: reset_index) {
        indices.emplace_back(torch::indexing::Slice(index));
    }
    obs_buffer_.index_put_(indices, new_obs.repeat({1, include_history_steps_}));
}

void ObservationBuffer::clear()
{
    obs_buffer_ = torch::zeros_like(obs_buffer_);
}

void ObservationBuffer::insert(const torch::Tensor &new_obs) {
    // Shift observations back.
    const torch::Tensor shifted_obs = obs_buffer_.index({
        torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(num_obs_, num_obs_ * include_history_steps_)
    }).clone();
    obs_buffer_.index({
        torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(0, num_obs_ * (include_history_steps_ - 1))
    }) = shifted_obs;

    // Add new observation.
    obs_buffer_.index({
        torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(-num_obs_, torch::indexing::None)
    }) = new_obs;
}

torch::Tensor ObservationBuffer::getObsVec(const std::vector<int> &obs_ids) const {
    std::vector<torch::Tensor> obs;
    for (int i = obs_ids.size() - 1; i >= 0; --i) {
        const int obs_id = obs_ids[i];
        const int slice_idx = include_history_steps_ - obs_id - 1;
        obs.push_back(obs_buffer_.index({
            torch::indexing::Slice(torch::indexing::None),
            torch::indexing::Slice(slice_idx * num_obs_, (slice_idx + 1) * num_obs_)
        }));
    }
    return cat(obs, -1);
}

torch::Tensor ObservationBuffer::getTermMajorObsVec(const std::vector<int>& term_dims) const
{
    int term_total = 0;
    for (const int dim : term_dims)
    {
        if (dim <= 0)
            throw std::runtime_error("Observation term dimensions must be positive");
        term_total += dim;
    }
    if (term_total != num_obs_)
        throw std::runtime_error("Observation term dimensions do not match one policy frame");

    std::vector<torch::Tensor> terms;
    terms.reserve(term_dims.size());
    int term_offset = 0;
    for (const int dim : term_dims)
    {
        std::vector<torch::Tensor> history;
        history.reserve(include_history_steps_);
        for (int frame = 0; frame < include_history_steps_; ++frame)
        {
            const int begin = frame * num_obs_ + term_offset;
            history.push_back(obs_buffer_.index({
                torch::indexing::Slice(torch::indexing::None),
                torch::indexing::Slice(begin, begin + dim)
            }));
        }
        terms.push_back(cat(history, -1));
        term_offset += dim;
    }
    return cat(terms, -1);
}
