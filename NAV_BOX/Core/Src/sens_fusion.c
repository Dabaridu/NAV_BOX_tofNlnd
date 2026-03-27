#include "sens_fusion.h"
#include <string.h>
#include <math.h>

/* ══════════════════════════════════════════
 *  Global variable definitions
 * ══════════════════════════════════════════ */
extern double Hd[4][4];

vector_t acc;
vector_t acc_prec  = {1e-5, 1e-5, 1e-5, 1e-5};
vector_t abs_q;

vector_t pos;
vector_t pos_prec = {1e-3, 1e-3, 1e-3, 1e-3};

vector_t X_global_translation;
vector_t X_global_translation_precision;
euler_t  O_global_orientation_euler;

kalman_state_t kf;

vector_t acc_f;
int ts = 0;
int lt = 0;

double starting_position_offset[3] = {0.0, 0.0, 0.0};
double starting_orientation_offset[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
};

/* ══════════════════════════════════════════
 *  Quaternion → 3×3 Rotation matrix
 * ══════════════════════════════════════════ */
void quaternion_to_rotation_3x3(vector_t *q, double R[3][3])
{
    double ww = q->w * q->w;
    double xx = q->x * q->x;
    double yy = q->y * q->y;
    double zz = q->z * q->z;
    double wx = q->w * q->x;
    double wy = q->w * q->y;
    double wz = q->w * q->z;
    double xy = q->x * q->y;
    double xz = q->x * q->z;
    double yz = q->y * q->z;

    R[0][0] = ww + xx - yy - zz;
    R[0][1] = 2.0 * (xy - wz);
    R[0][2] = 2.0 * (xz + wy);

    R[1][0] = 2.0 * (xy + wz);
    R[1][1] = ww - xx + yy - zz;
    R[1][2] = 2.0 * (yz - wx);

    R[2][0] = 2.0 * (xz - wy);
    R[2][1] = 2.0 * (yz + wx);
    R[2][2] = ww - xx - yy + zz;
}

/* ══════════════════════════════════════════
 *  Gimbal-lock safe Euler extraction
 *
 *  Convention: ZYX (yaw → pitch → roll)
 *
 *  Normal:
 *    pitch = asin(-R[2][0])
 *    roll  = atan2(R[2][1], R[2][2])
 *    yaw   = atan2(R[1][0], R[0][0])
 *
 *  Gimbal lock (|R[2][0]| ≈ 1):
 *    pitch = ±90°
 *    roll  = 0  (arbitrarily)
 *    yaw   = atan2(-R[0][1], R[1][1])
 * ══════════════════════════════════════════ */
euler_t rotation_to_euler_safe(double R[3][3])
{
    euler_t e;
    double sin_pitch = -R[2][0];

    /* Clamp for numerical safety */
    if (sin_pitch > 1.0)  sin_pitch = 1.0;
    if (sin_pitch < -1.0) sin_pitch = -1.0;

    e.pitch = asin(sin_pitch);

    double cos_pitch = cos(e.pitch);

    if (fabs(cos_pitch) < GIMBAL_LOCK_THRESHOLD) {
        /* ── Gimbal lock ── */
        e.gimbal_locked = 1;
        e.roll  = 0.0;  // set roll to zero by convention
        e.yaw   = atan2(-R[0][1], R[1][1]);
    } else {
        /* ── Normal case ── */
        e.gimbal_locked = 0;
        e.roll  = atan2(R[2][1], R[2][2]);
        e.yaw   = atan2(R[1][0], R[0][0]);
    }
    return e;
}

/* ══════════════════════════════════════════
 *  IMU position + orientation update
 *  (standalone, without Kalman)
 * ══════════════════════════════════════════ */
void update_IMU_global_position(
    double sp[3], double so[3][3],      /*position and orientation offsets*/
    vector_t *acc_in, vector_t *abs_q_in,
    int time_step,
    vector_t *X_global_translation,
    euler_t  *O_global_orientation_euler)
{
    /* Velocity must persist between calls */
    static vector_t velocity = {0.0, 0.0, 0.0, 0.0};

    double dt = time_step / 1000000.0;
    if (dt <= 0.0) return;

    /* ── 1. Absolute rotation from quaternion ── */
    double R_abs[3][3];
    quaternion_to_rotation_3x3(abs_q_in, R_abs);

    /* ── 2. Relative rotation:  R_rel = R_abs × so^T
     *       (remove starting orientation)            ── */
    double R_rel[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_rel[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                // so is orthogonal → so^T = so inverse
                R_rel[i][j] += R_abs[i][k] * so[j][k];
            }
        }
    }

    /* ── 3. Rotate acceleration into global frame ── */
    double ax_g = R_rel[0][0]*acc_in->x
                + R_rel[0][1]*acc_in->y
                + R_rel[0][2]*acc_in->z;
    double ay_g = R_rel[1][0]*acc_in->x
                + R_rel[1][1]*acc_in->y
                + R_rel[1][2]*acc_in->z;
    double az_g = R_rel[2][0]*acc_in->x
                + R_rel[2][1]*acc_in->y
                + R_rel[2][2]*acc_in->z;

    /* ── 4. Double integration ──
     *
     *  Physics (constant-acceleration model within dt):
     *
     *    v(t+dt) = v(t) + a · dt          ← 1st integral
     *    x(t+dt) = x(t) + v(t)·dt + ½a·dt²  ← 2nd integral
     *
     *  Note: position uses v(t) BEFORE update,
     *        so we update position first, then velocity.
     */
    X_global_translation->x += velocity.x * dt + 0.5 * ax_g * dt * dt;
    X_global_translation->y += velocity.y * dt + 0.5 * ay_g * dt * dt;
    X_global_translation->z += velocity.z * dt + 0.5 * az_g * dt * dt;

    velocity.x += ax_g * dt;
    velocity.y += ay_g * dt;
    velocity.z += az_g * dt;

    /* Apply starting position offset */
    X_global_translation->x += sp[0];
    X_global_translation->y += sp[1];
    X_global_translation->z += sp[2];
    // Note: offset should only be applied once at init,
    // or stored separately. This is kept for compatibility.

    /* ── 5. Euler angles (gimbal-lock safe) ── */
    *O_global_orientation_euler = rotation_to_euler_safe(R_rel);
}

/* ══════════════════════════════════════════
 *  Moving average filter
 * ══════════════════════════════════════════ */
void vector_t_moving_average_filter(vector_t *input, vector_t *output)
{
    static vector_t buffer[RUNING_AVG_BUFF];
    static int index = 0;

    buffer[index] = *input;
    index = (index + 1) % RUNING_AVG_BUFF;

    output->x = 0.0;
    output->y = 0.0;
    output->z = 0.0;

    for (int i = 0; i < RUNING_AVG_BUFF; i++) {
        output->x += buffer[i].x;
        output->y += buffer[i].y;
        output->z += buffer[i].z;
    }
    output->x /= RUNING_AVG_BUFF;
    output->y /= RUNING_AVG_BUFF;
    output->z /= RUNING_AVG_BUFF;
}

/* ══════════════════════════════════════════
 *  4×4 Matrix multiply (kept for debug)
 * ══════════════════════════════════════════ */
void matrix_multiply(double A[4][4], double B[4][4], double result[4][4])
{
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
                result[i][j] += A[i][k] * B[k][j];
        }
}

/* ══════════════════════════════════════════════════════
 *  KALMAN FILTER — 9-state (pos, vel, acc)
 *
 *  State:  x = [px py pz  vx vy vz  ax ay az]^T
 *
 *  State transition (constant acceleration model):
 *
 *     p(k+1) = p(k) + v(k)·dt + ½·a(k)·dt²
 *     v(k+1) = v(k) + a(k)·dt
 *     a(k+1) = a(k)              ← assumed constant
 *
 *  IMU predict:  uses rotated acceleration as
 *                direct measurement of a(k),
 *                injected into state before propagation.
 *
 *  GPS update:   measures [px py pz].
 * ══════════════════════════════════════════════════════ */

/* ── Helper: 9×9 matrix operations ── */

static void mat9_zero(double M[KF_STATES][KF_STATES])
{
    memset(M, 0, sizeof(double) * KF_STATES * KF_STATES);
}

static void mat9_identity(double M[KF_STATES][KF_STATES])
{
    mat9_zero(M);
    for (int i = 0; i < KF_STATES; i++) M[i][i] = 1.0;
}

static void mat9_multiply(double A[KF_STATES][KF_STATES],
                           double B[KF_STATES][KF_STATES],
                           double C[KF_STATES][KF_STATES])
{
    double tmp[KF_STATES][KF_STATES];
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_STATES; j++) {
            tmp[i][j] = 0.0;
            for (int k = 0; k < KF_STATES; k++)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    memcpy(C, tmp, sizeof(tmp));
}

static void mat9_add(double A[KF_STATES][KF_STATES],
                      double B[KF_STATES][KF_STATES],
                      double C[KF_STATES][KF_STATES])
{
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_STATES; j++)
            C[i][j] = A[i][j] + B[i][j];
}

static void mat9_transpose(double A[KF_STATES][KF_STATES],
                             double AT[KF_STATES][KF_STATES])
{
    double tmp[KF_STATES][KF_STATES];
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_STATES; j++)
            tmp[i][j] = A[j][i];
    memcpy(AT, tmp, sizeof(tmp));
}

/* ── 3×3 matrix inverse (for GPS update) ── */
static int mat3_invert(double A[3][3], double Ainv[3][3])
{
    double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
               - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
               + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabs(det) < 1e-15) return -1; // singular

    double inv_det = 1.0 / det;

    Ainv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * inv_det;
    Ainv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * inv_det;
    Ainv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * inv_det;
    Ainv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * inv_det;
    Ainv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * inv_det;
    Ainv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * inv_det;
    Ainv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * inv_det;
    Ainv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * inv_det;
    Ainv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * inv_det;

    return 0;
}

/* ══════════════════════════════════════════
 *  Kalman Init
 * ══════════════════════════════════════════ */
void kalman_init(kalman_state_t *kf,
                 double imu_acc_var,
                 double gps_pos_var,
                 double initial_time_s)
{
    memset(kf, 0, sizeof(kalman_state_t));

    /* Initial covariance — large uncertainty */
    for (int i = 0; i < KF_STATES; i++)
        kf->P[i][i] = 1000.0;

    /* Process noise:
     *  position states:     low  (driven by model)
     *  velocity states:     medium
     *  acceleration states: high (can change quickly)
     */
    kf->Q[0][0] = 0.01;   kf->Q[1][1] = 0.01;   kf->Q[2][2] = 0.01;
    kf->Q[3][3] = 0.1;    kf->Q[4][4] = 0.1;     kf->Q[5][5] = 0.1;
    kf->Q[6][6] = 1.0;    kf->Q[7][7] = 1.0;     kf->Q[8][8] = 1.0;

    /* GPS measurement noise */
    kf->R_gps[0][0] = gps_pos_var;
    kf->R_gps[1][1] = gps_pos_var;
    kf->R_gps[2][2] = gps_pos_var;

    kf->imu_acc_variance = imu_acc_var;
    kf->last_time_s = initial_time_s;
    kf->initialized = 1;
}

/* ══════════════════════════════════════════
 *  Kalman Predict (IMU accelerometer)
 *
 *  Called every IMU sample (~100-1000 Hz)
 *
 *  acc_global: acceleration already rotated
 *              to global frame with gravity removed
 * ══════════════════════════════════════════ */
void kalman_predict_imu(kalman_state_t *kf,
                        vector_t *acc_global,
                        double dt)
{
    if (dt <= 0.0) return;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    /* ── Inject IMU acceleration into state ── */
    kf->x[6] = acc_global->x;
    kf->x[7] = acc_global->y;
    kf->x[8] = acc_global->z;

    /* ── State transition matrix F ──
     *
     *  F = | I   I·dt   ½I·dt² |   (pos row)
     *      | 0    I     I·dt   |   (vel row)
     *      | 0    0      I     |   (acc row)
     */
    double F[KF_STATES][KF_STATES];
    mat9_identity(F);

    // pos += vel*dt
    F[0][3] = dt;  F[1][4] = dt;  F[2][5] = dt;
    // pos += 0.5*acc*dt²
    F[0][6] = 0.5*dt2; F[1][7] = 0.5*dt2; F[2][8] = 0.5*dt2;
    // vel += acc*dt
    F[3][6] = dt;  F[4][7] = dt;  F[5][8] = dt;

    /* ── Propagate state: x = F·x ── */
    double x_new[KF_STATES] = {0};
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_STATES; j++)
            x_new[i] += F[i][j] * kf->x[j];
    memcpy(kf->x, x_new, sizeof(x_new));

    /* ── Process noise scaled by dt ──
     *
     *  Continuous white noise acceleration model:
     *  Q_discrete ≈ G·q·G^T where G maps acc noise to states
     *
     *  Simplified diagonal scaling:
     */
    double Q_scaled[KF_STATES][KF_STATES];
    mat9_zero(Q_scaled);

    double q_a = kf->imu_acc_variance;

    // Position variance from acceleration noise
    Q_scaled[0][0] = q_a * dt4 / 4.0;
    Q_scaled[1][1] = q_a * dt4 / 4.0;
    Q_scaled[2][2] = q_a * dt4 / 4.0;
    // Velocity variance from acceleration noise
    Q_scaled[3][3] = q_a * dt2;
    Q_scaled[4][4] = q_a * dt2;
    Q_scaled[5][5] = q_a * dt2;
    // Acceleration process noise
    Q_scaled[6][6] = kf->Q[6][6] * dt;
    Q_scaled[7][7] = kf->Q[7][7] * dt;
    Q_scaled[8][8] = kf->Q[8][8] * dt;

    // Cross terms (pos-vel)
    double pv_cross = q_a * dt3 / 2.0;
    Q_scaled[0][3] = pv_cross; Q_scaled[3][0] = pv_cross;
    Q_scaled[1][4] = pv_cross; Q_scaled[4][1] = pv_cross;
    Q_scaled[2][5] = pv_cross; Q_scaled[5][2] = pv_cross;

    /* ── Propagate covariance: P = F·P·F^T + Q ── */
    double FT[KF_STATES][KF_STATES];
    double FP[KF_STATES][KF_STATES];
    double FPFT[KF_STATES][KF_STATES];

    mat9_transpose(F, FT);
    mat9_multiply(F, kf->P, FP);
    mat9_multiply(FP, FT, FPFT);
    mat9_add(FPFT, Q_scaled, kf->P);
}

/* ══════════════════════════════════════════
 *  Kalman Update (GPS measurement)
 *
 *  Called every GPS fix (~1-10 Hz)
 *
 *  H = [I₃ₓ₃  0₃ₓ₃  0₃ₓ₃]  (measures position only)
 *
 *  Standard Kalman update:
 *    y = z - H·x
 *    S = H·P·H^T + R
 *    K = P·H^T·S⁻¹
 *    x = x + K·y
 *    P = (I - K·H)·P
 * ══════════════════════════════════════════ */
void kalman_update_gps(kalman_state_t *kf,
                       double gps_x, double gps_y, double gps_z)
{
    /* ── Innovation y = z - H·x ── */
    double y[KF_POS_MEAS];
    y[0] = gps_x - kf->x[0];
    y[1] = gps_y - kf->x[1];
    y[2] = gps_z - kf->x[2];

    /* ── S = H·P·H^T + R  (3×3)
     *    Since H = [I₃ 0₃ 0₃], H·P·H^T = P[0:2][0:2] ── */
    double S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = kf->P[i][j] + kf->R_gps[i][j];

    /* ── S⁻¹ ── */
    double S_inv[3][3];
    if (mat3_invert(S, S_inv) != 0) return; // singular, skip update

    /* ── K = P·H^T·S⁻¹  (9×3)
     *    P·H^T = P[:][0:2] ── */
    double K[KF_STATES][KF_POS_MEAS];
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_POS_MEAS; j++) {
            K[i][j] = 0.0;
            for (int k = 0; k < KF_POS_MEAS; k++)
                K[i][j] += kf->P[i][k] * S_inv[k][j];
        }

    /* ── x = x + K·y ── */
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_POS_MEAS; j++)
            kf->x[i] += K[i][j] * y[j];

    /* ── P = (I - K·H)·P ──
     *    K·H is 9×9 but only columns 0-2 are nonzero ── */
    double KH[KF_STATES][KF_STATES];
    mat9_zero(KH);
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_POS_MEAS; j++)
            KH[i][j] = K[i][j];  // H is identity in first 3 cols

    double I_KH[KF_STATES][KF_STATES];
    mat9_identity(I_KH);
    for (int i = 0; i < KF_STATES; i++)
        for (int j = 0; j < KF_STATES; j++)
            I_KH[i][j] -= KH[i][j];

    double P_new[KF_STATES][KF_STATES];
    mat9_multiply(I_KH, kf->P, P_new);
    memcpy(kf->P, P_new, sizeof(P_new));
}

/* ── Getters ── */
void kalman_get_position(kalman_state_t *kf, vector_t *out)
{
    out->w = 0.0;
    out->x = kf->x[0];
    out->y = kf->x[1];
    out->z = kf->x[2];
}

void kalman_get_velocity(kalman_state_t *kf, vector_t *out)
{
    out->w = 0.0;
    out->x = kf->x[3];
    out->y = kf->x[4];
    out->z = kf->x[5];
}

/* ══════════════════════════════════════════
 *  Timestamp utility
 * ══════════════════════════════════════════ */
double update_ts(int *last_time, int current_time)
{
    int time_step = current_time - *last_time;
    *last_time = current_time;
    return (double)time_step;
}