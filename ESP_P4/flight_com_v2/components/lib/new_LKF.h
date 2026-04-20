void new_state_extrapolation(float* state, float* s_tran, float* control);

void new_covariance_extrapolation(float* s_tran, float* cov, float* process_noise_covariance);

void new_update_gain(float* gain, float* cov, float measurement_uncertainty);

void new_state_update(float* state, float* gain, float measurement);

void new_covariance_update(float* cov, float* gain, float measurement_uncertainty);