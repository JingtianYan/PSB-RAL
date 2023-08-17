#include "../include/berstein.h"

BernsteinPolynomial::BernsteinPolynomial(int user_control_points, double time_duration)
{
    T = time_duration;
    // control_points = user_control_points;
    N = user_control_points-1;
    for (int i = 0; i <= N; i++) {
        control_points_deriv.orignal_idx.push_back(std::vector<int> {i});
        control_points_deriv.sign_idx.push_back(std::vector<int> {1});
    }
    control_points_deriv.N = N;
    control_points_deriv.deriv_order = 1;
    control_points_deriv.ratio = 1.0;
}

BernsteinPolynomial::BernsteinPolynomial(int user_control_points, double time_duration, BernDeriv curr_deriv)
{
    T = time_duration;
    N = user_control_points-1;
    control_points_deriv = curr_deriv;
}

/**
 * Utility function to calculate choose r from n
 * 
 * Since c++ do not have built-in factorial function, use tgamma instead
 * 
 * @param n Total class
 * @param r The number of choose items
 * @return The number of all the situations that could happen
*/
long double BernsteinPolynomial::ChooseNFromR(int n, int r)
{
    long double result = tgamma(n+1)/(tgamma(r+1)*tgamma(n-r+1));
    return result;
}
// long double BernsteinPolynomial::ChooseNFromR(int n, int k)
// {
//     if (k > n) return 0;
//     if (k * 2 > n) k = n-k;
//     if (k == 0) return 1;

//     long double result = n;
//     for( int i = 2; i <= k; ++i ) {
//         result *= (n-i+1);
//         result /= i;
//     }
//     return result;
// }

/**
 * Base function of Bernstein Polynomial
 * 
 * @param idx Index of the Bernstein Polynomial
 * @param t Input value of Bernstein Polynomial
 * @return Value the Bernstein
*/
double BernsteinPolynomial::Berstein(int idx, double t)
{
    // printf("C from N R is: %Lf\n", ChooseNFromR(N, idx));
    if (t == 0.0 && idx == 0) {
        return ChooseNFromR(N, idx);
    } else if (t == T && idx == N) {
        return ChooseNFromR(N, idx);
    }
    return ChooseNFromR(N, idx) * pow(t, idx) * pow(T-t, N-idx) / pow(T, N);
}

/**
 * Calculate the value of the Bernstein Polynomial
 * 
 * @param t Input value for Bernstein Polynomial
 * @return The calculated value
*/
bool BernsteinPolynomial::CalculateVal(double t, std::vector<double>& val_vec)
{
    for (int i = 0; i <= N; i++){
        val_vec.push_back(Berstein(i, t));
    }
    return true;
}

double BernsteinPolynomial::GetVal(double t, std::vector<double>& control_points)
{
    double total_val = 0.0;
    for (int i = 0; i <= N; i++){
        total_val = total_val + Berstein(i, t)*control_points[i];
        // std::cout << "berstein: " << Berstein(i, t) << std::endl;
    }
    return total_val;
}


/**
 * Derivate of the Berstein Polynomial
 * 
 * TODO: redesign this function
*/
BernsteinPolynomial BernsteinPolynomial::Derivative()
{
    // std::vector<double> deriv_control_points;
    // for (int i = 0; i < N; i++){
    //     double tmp_control_point = ((control_points[i+1] - control_points[i]) * N) / T;
    //     deriv_control_points.push_back(tmp_control_point);
    // }
    BernDeriv tmp_deriv;
    for (int i = 0; i < N; i++){
        // control_points[i+1] - control_points[i]
        std::vector<int> tmp_idx_vec;
        tmp_idx_vec.insert(tmp_idx_vec.end(), control_points_deriv.orignal_idx[i].begin(), control_points_deriv.orignal_idx[i].end());
        tmp_idx_vec.insert(tmp_idx_vec.end(), control_points_deriv.orignal_idx[i+1].begin(), control_points_deriv.orignal_idx[i+1].end());
        tmp_deriv.orignal_idx.push_back(tmp_idx_vec);

        // std::vector<int> tmp_sign_vec;
        // tmp_sign_vec.insert(tmp_sign_vec.end(), control_points_deriv.sign_idx[i].begin(), control_points_deriv.sign_idx[i].end());
        // for (int tmp_sign: control_points_deriv.sign_idx[i+1]) {
        //     tmp_sign_vec.push_back(-tmp_sign);
        // }
        // tmp_deriv.sign_idx.push_back(tmp_sign_vec);
        std::vector<int> tmp_sign_vec;
        for (int tmp_sign: control_points_deriv.sign_idx[i]) {
            tmp_sign_vec.push_back(-tmp_sign);
        }
        tmp_sign_vec.insert(tmp_sign_vec.end(), control_points_deriv.sign_idx[i+1].begin(), control_points_deriv.sign_idx[i+1].end());
        tmp_deriv.sign_idx.push_back(tmp_sign_vec);
    }

    tmp_deriv.N = N - 1;
    tmp_deriv.ratio = control_points_deriv.ratio * N / T;
    tmp_deriv.deriv_order = control_points_deriv.deriv_order + 1;
    return BernsteinPolynomial(N, T, tmp_deriv);
}