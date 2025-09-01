#pragma once

namespace Lionheart::LeastSquaresSolver {

template <typename T> using Matrix6x5 = T[6][5];
template <typename T> using Matrix5x6 = T[5][6];
template <typename T> using Matrix5x5 = T[5][5];
template <typename T> using Vector5 = T[5];
template <typename T> using Vector6 = T[6];

template <typename T>
void solveLinearSystem(const Matrix5x5<T> &A_in, const Vector5<T> &b_in,
                       Vector5<T> &x) {
    // Make copies, since we'll modify A and b
    Matrix5x5<T> A = A_in;
    Vector5<T> b = b_in;

    const int N = 5;

    // Forward elimination
    for (int i = 0; i < N; ++i) {
        // Partial pivoting: find the row with the largest element in this
        // column
        int pivotRow = i;
        double maxVal = std::abs(A[i][i]);
        for (int k = i + 1; k < N; ++k) {
            if (std::abs(A[k][i]) > maxVal) {
                maxVal = std::abs(A[k][i]);
                pivotRow = k;
            }
        }

        // Swap rows in A and b
        if (pivotRow != i) {
            std::swap(A[i], A[pivotRow]);
            std::swap(b[i], b[pivotRow]);
        }

        // Eliminate column i for all rows below
        for (int k = i + 1; k < N; ++k) {
            double factor = A[k][i] / A[i][i];
            for (int j = i; j < N; ++j) {
                A[k][j] -= factor * A[i][j];
            }
            b[k] -= factor * b[i];
        }
    }

    // Back substitution
    for (int i = N - 1; i >= 0; --i) {
        double sum = 0.0;
        for (int j = i + 1; j < N; ++j) {
            sum += A[i][j] * x[j];
        }
        x[i] = (b[i] - sum) / A[i][i];
    }

    return x;
}

/// @brief Solve least squares for Ax = b
/// @tparam T type
/// @param A 6x5 matrix
/// @param b 6x1 vector
/// @param x 5x1 vector
template <typename T>
void solve_least_squares(const Matrix6x5<T> &A, const Vector6<T> &b,
                         Vector5<T> &x) {

    // Compute A transpose.
    Matrix5x6<T> At;
    for (uint32_t r = 0; r < 6; r++) {
        for (uint32_t c = 0; c < 5; c++) {
            At[c][r] = A[r][c];
        }
    }

    // Compute A transpose * A.
    Matrix5x5<T> AtA;
    memset(AtA, 0, sizeof(AtA));
    for (uint32_t AtA_r = 0; AtA_r < 5; AtA_r++) {
        for (uint32_t AtA_c = 0; AtA_c < 5; AtA_c++) {
            for (uint32_t i = 0; i < 6; ++i) {
                AtA[AtA_r][AtA_c] += At[AtA_r][i] * A[i][AtA_c];
            }
        }
    }

    // Compute Atb
    Vector5<T> Atb;
    memset(Atb, 0, sizeof(Atb));
    for (uint32_t r = 0; r < 5; r++) {
        for (uint32_t c = 0; c < 5; c++) {
            Atb[r] += At[r][c] * b[c];
        }
    }

    // Solve (AtA)x = (At)b
    solveLinearSystem(AtA, b, x);
}

} // namespace Lionheart::LeastSquaresSolver