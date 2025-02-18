#include <math.h>

void mat21_multiply_mat12(float A[2][1], float B[1][2], float C[2][2]) {
    C[0][0] = A[0][0] * B[0][0];
    C[0][1] = A[0][0] * B[0][1];
    C[1][0] = A[1][0] * B[0][0];
    C[1][1] = A[1][0] * B[0][1];
}

void mat_elemwise_multiply22D(float A[2][2], float B, float C[2][2]) {
    C[0][0] = A[0][0] * B;
    C[0][1] = A[0][1] * B;
    C[1][0] = A[1][0] * B;
    C[1][1] = A[1][1] * B;
}

void mat_elemwise_multiply21D(float A[2][1], float B, float C[2][1]) {
    C[0][0] = A[0][0] * B;
    C[1][0] = A[1][0] * B;
}

void mat_vec_multiply2D(float A[2][2], float B[2][1], float C[2][1]) {
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
}

void vec_add2D(float A[2][1], float B[2][1], float C[2][1]) {
    C[0][0] = A[0][0] + B[0][0];
    C[1][0] = A[1][0] + B[1][0];
}

void mat_add22D(float A[2][2], float B[2][2], float C[2][2]) {
    C[0][0] = A[0][0] + B[0][0];
    C[1][0] = A[1][0] + B[1][0];
    C[0][1] = A[0][1] + B[0][1];
    C[1][1] = A[1][1] + B[1][1];
}

void mat_multiply22D(const float A[2][2], const float B[2][2], float C[2][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void compute_FPFt(const float F[2][2], const float P[2][2], float P_1[2][2]) {
    float FP[2][2];   // Intermediate matrix FP
    float Ft[2][2];   // Transpose of F

    mat_multiply22D(F, P, FP);

    Ft[0][0] = F[0][0]; Ft[0][1] = F[1][0];
    Ft[1][0] = F[0][1]; Ft[1][1] = F[1][1];

    mat_multiply22D(FP, Ft, P_1);
}

void kalman_filter_fusion(float dt_us, float a_imu, float z_baro, float acc_std, float R, float s_out[2][1]) {
    // Convert time to seconds
    float dt = dt_us * 1e-6;

    // State vector and covariance matrix (persistent between calls)
    static float s[2][1] = {{0}, {0}};
    static float P[2][2] = {{0, 0}, {0, 0}};

    // State transition matrix
    const float F[2][2] = {{1, dt}, {0, 1}};

    // Control input matrix
    const float G[2][1] = {{0.5 * pow(dt, 2)}, {dt}};

    // Measurement matrix
    const float H[1][2] = {1, 0};

    // Process noise covariance matrix
    float Q[2][2];
    float G_T[1][2] = {G[0][0], G[1][0]};
    mat21_multiply_mat12(G, G_T, Q); // Q = G * G^T
    float acc_std2 = pow(acc_std, 2);
    mat_elemwise_multiply22D(Q, acc_std2, Q); // Q = Q * acc_std^2

    // Prediction step
    float s_1[2][1], s_2[2][1];
    mat_vec_multiply2D(F, s, s_1); // s_1 = F * s
    mat_elemwise_multiply21D(G, a_imu, s_2); // s_2 = G * a_imu
    vec_add2D(s_1, s_2, s); // s = s_1 + s_2

    float P_1[2][2];
    compute_FPFt(F, P, P_1); // P_1 = F * P * F^T
    mat_add22D(P_1, Q, P); // P = P_1 + Q

    // Update step
    float L = H[0][0] * P[0][0] * H[0][0] + R; // L = H * P * H^T + R

    float K[2][1];
    float H_T[2][1] = {H[0][0], H[1][0]};
    mat_vec_multiply2D(P, H_T, K); // K = P * H^T
    K[0][0] /= L; // K = P * H^T / L
    K[1][0] /= L;

    float Hs = H[0][0] * s[0][0]; // Hs = H * s
    float Innov = z_baro - Hs; // Innov = z_baro - H * s

    float delta_s[2][1];
    mat_elemwise_multiply21D(K, Innov, delta_s); // delta_s = K * Innov
    vec_add2D(s, delta_s, s); // s = s + delta_s

    // Update covariance matrix
    float KH[2][2] = {{K[0][0] * H[0][0], K[0][0] * H[0][1]},
                     {K[1][0] * H[0][0], K[1][0] * H[0][1]}};
    float I_minus_KH[2][2] = {{1 - KH[0][0], -KH[0][1]},
                              {-KH[1][0], 1 - KH[1][1]}};
    float P_new[2][2];
    mat_multiply22D(I_minus_KH, P, P); // P = (I - K * H) * P

    // Copy output
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 1; j++) {
            s_out[i][j] = s[i][j];
        }
    }
}