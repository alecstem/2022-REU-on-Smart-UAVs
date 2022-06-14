"""
Contains functions for path smoothing (Bezier curve).
Code modified from https://github.com/rparak/Bezier_Curve/
"""
import numpy as np

def Binomial_Coefficient(n, k):
    """
    Description:
        Calculation binomial coofecient C, from pair of integers n ≥ k ≥ 0 and is written (n k). The binomial coefficients are the
        positive integers that occur as coefficients in the binomial theorem.
        (n k) = n! / (k! * (n - k)!)
        ...
        Simplification of the calculation:
        (n k) = ((n - k + 1) * (n - k + 2) * ... * (n - 1) * (n)) / (1 * 2 * ... * (k - 1) * k)

    Args:
        (1) n [INT]: Integer number 1 (numerator)
        (2) k [INT]: Integer number 2 (denumerator)
    Returns:
        (1) parameter [INT]: Binomial coofecient C(n k).
    """

    try:
        assert (n >= k)

        if k == 0:
            return 1
        elif k == 1:
            return n
        else:
            c_nk = 1
            # Calculation from the simplification equation
            for i in range(0, k):
                # Numerator and Denumerator.
                c_nk *= (n - i);
                c_nk /= (i + 1)

            return c_nk

    except AssertionError as error:
        print('[ERROR] The number n must be larger than or equal to k.')
        print(f'[ERROR] Information: {error}')

# Initialization of constants
CONST_NUM_OF_ENTRY_POINTS_LINEAR = 2
CONST_NUM_OF_ENTRY_POINTS_QUADRATIC = 3
CONST_NUM_OF_ENTRY_POINTS_CUBIC = 4
# Time t ∈ [0: The starting value of the sequence,
#           1: The end value of the sequence]
CONST_T_START = 0
CONST_T_STOP = 1

class N_Degree(object):
    """
    Description:
        Class for efficient solution of N-degree Bézier curve.
        Note:
            A Bézier curve is a parametric curve used in computer graphics and related fields.
    Initialization of the Class:
        Input:
            (1) num_of_samples [INT]: Number of samples to generate. Must be non-negative.
    Example:
        Initialization:
            Cls = N_Degree(num_of_samples)
        Calculation:
            res = Cls.Solve(points)

            where p is equal to [[px_id_0, py_id_0], .., [px_id_n, py_id_n]] in 2D space
            and [[px_id_0, py_id_0, pz_id_0], .., [px_id_n, py_id_n, pz_id_n]] in 3D space
    """

    def __init__(self, num_of_samples):
        # << PUBLIC >> #
        try:
            assert (num_of_samples >= 0)
            # Return evenly spaced numbers over a specified interval.
            self.t = np.linspace(CONST_T_START, CONST_T_STOP, num_of_samples)

        except AssertionError as error:
            print('[ERROR] The number of samples must not be negative.')

        # << PRIVATE >> #
        # Number of samples to generate
        self.__num_of_samples = num_of_samples

    def __n_index_curve(self, i, point, n, c_ni):
        """
        Description:
            Given n + 1 control points p_{0}, p_{1},..., p_{n} we define the degree n Bezier curve to
            be the curve parametrized by (De Casteljau's algorithm):
            p(t) = sum(i = 0 -> n) (C(n i)) * (t ^ i) * ((1 - t) ^ (n - i)) * p_{i}, t ∈ [0, 1]
            where C(n i) is a binomial coefficient.
        Args:
            (1) i [INT]: Iteration.
            (2) point [Int/Float Matrix]: Point (2D/3D) in interation (i).
            (3) n [INT]: Number of points.
            (4) c_ni [INT]: Binomial coofecient C(n i) in iteration (i).
        Returns:
            (1) parameter [Int/Float Matrix]: Results of curve values in iteration (i).
        """

        return [c_ni * (self.t ** i) * ((1 - self.t) ** (n - i)) * p
                for _, p in enumerate(point)]

    def __n_degree(self, points):
        """
        Description:
            The main control function for creating a Bézier curve of degree n.
        Returns:
            (1) parameter [{0 .. Number of dimensions - 1}] [Int/Float Matrix]: Resulting points of the curve.
        """

        # Number of points in the matrix
        n = len(points) - 1

        # Calculation of binomial cooficient of the first iteration
        c_nk = Binomial_Coefficient(n, 0)
        # Calculation of the first curve positions
        result = self.__n_index_curve(0, points[0], n, c_nk)

        for i in range(1, n + 1):
            # Binomial cooficient in interation (i)
            c_ni = Binomial_Coefficient(n, i)

            # Calculation positions in iteration (i)
            aux_result = self.__n_index_curve(i, points[i], n, c_ni)

            # The sum of all positions for the resulting Bézier curve
            for j in range(0, len(aux_result)):
                result[j] += aux_result[j]

        return result

    def Solve(self, points):
        """
        Description:
            Function for automatic calculation of a suitably selected Bézier curve.
        Args:
            (1) points [p_{0, .., n}] [Int/Float Matrix]: Multiple points to create a curve.
        Return:
            (1) parameter [Int/Float Matrix]: Resulting points of the curve.
        """

        try:
            assert len(points) > 1
            # Selects the calculation method based on the number of points in the matrix (p).
            return self.__n_degree(points)

        except AssertionError as error:
            print('[ERROR] Insufficient number of entry points.')
            print(f'[ERROR] The minimum number of entry points is {CONST_NUM_OF_ENTRY_POINTS_LINEAR}.')
            print(f'[ERROR] Information: {error}')


def calculate_bezier(path, num_of_samples):
    points = []
    BezierNDegree = N_Degree(num_of_samples)
    result = BezierNDegree.Solve(path)
    for k in range(len(result[0])):
        points.append([result[0][k], result[1][k], result[2][k]])
    return points
