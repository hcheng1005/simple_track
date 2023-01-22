
#ifndef ASSOCIATION_H
#define ASSOCIATION_H

#include "iostream"
#include <vector>

// used for "Greedy_solver"
typedef std::pair<std::pair<uint32_t, uint32_t>, double> cost_pair;

// used for "Hungarian_solver"
typedef float track_t;
typedef std::vector<int> assignments_t;
typedef std::vector<track_t> distMatrix_t;

// used for "LAP_solver"
#define LARGE 1000000

#if !defined TRUE
#define TRUE 1
#endif
#if !defined FALSE
#define FALSE 0
#endif

#define NEW(x, t, n) if ((x = (t *)malloc(sizeof(t) * (n))) == 0) { return -1; }
#define FREE(x) if (x != 0) { free(x); x = 0; }
#define SWAP_INDICES(a, b) { int_t _temp_index = a; a = b; b = _temp_index; }

#if 0
#include <assert.h>
#define ASSERT(cond) assert(cond)
#define PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define PRINT_COST_ARRAY(a, n) \
    while (1) { \
        printf(#a" = ["); \
        if ((n) > 0) { \
            printf("%f", (a)[0]); \
            for (uint_t j = 1; j < n; j++) { \
                printf(", %f", (a)[j]); \
            } \
        } \
        printf("]\n"); \
        break; \
    }
#define PRINT_INDEX_ARRAY(a, n) \
    while (1) { \
        printf(#a" = ["); \
        if ((n) > 0) { \
            printf("%d", (a)[0]); \
            for (uint_t j = 1; j < n; j++) { \
                printf(", %d", (a)[j]); \
            } \
        } \
        printf("]\n"); \
        break; \
    }
#else
#define ASSERT(cond)
#define PRINTF(fmt, ...)
#define PRINT_COST_ARRAY(a, n)
#define PRINT_INDEX_ARRAY(a, n)
#endif

typedef signed int   int_t;
typedef unsigned int uint_t;
typedef double       cost_t;
typedef char         boolean;
typedef enum fp_t {
    FP_1 = 1,
    FP_2 = 2,
    FP_DYNAMIC = 3
}fp_t;
/* END for "lAP_solver*/

class Linear_Assigment
{
public:
    Linear_Assigment() = default;
    ~Linear_Assigment() = default;
    void greedy_solver(std::vector<std::vector<cost_pair>> &cost_matrix,
                     std::vector<int> &trace_assign_info,
                     std::vector<int> &meas_assign_info);

    enum HM_TMethod
    {
        optimal,
        many_forbidden_assignments,
        without_forbidden_assignments
    };

    track_t Hungarian_solver(const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns, assignments_t& assignment, HM_TMethod Method = optimal);

    int lapjv_internal(const uint_t n, cost_t *cost[], int_t *x, int_t *y);

    void auction(int N, double **cost, std::vector<int>& assignment);

private:
    // Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
    void assignmentoptimal(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns);
    void buildassignmentvector(assignments_t& assignment, bool *starMatrix, size_t nOfRows, size_t nOfColumns);
    void computeassignmentcost(const assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows);
    void step2a(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
    void step2b(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
    void step3_5(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
    void step4(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col);

    // Computes a suboptimal solution. Good for cases with many forbidden assignments.
    void assignmentsuboptimal1(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns);
    // Computes a suboptimal solution. Good for cases with many forbidden assignments.
    void assignmentsuboptimal2(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns);

    std::vector<track_t> m_distMatrix;

    static constexpr bool HUNGARIAN_LOGS = false;

    int_t _ccrrt_dense(const uint_t n, cost_t *cost[],
                        int_t *free_rows, int_t *x, int_t *y, cost_t *v);
    int_t _carr_dense(
        const uint_t n, cost_t *cost[],
        const uint_t n_free_rows,
        int_t *free_rows, int_t *x, int_t *y, cost_t *v);

    uint_t _find_dense(const uint_t n, uint_t lo, cost_t *d, int_t *cols, int_t *y);
    int_t _scan_dense(const uint_t n, cost_t *cost[],
                      uint_t *plo, uint_t *phi,
                      cost_t *d, int_t *cols, int_t *pred,
                      int_t *y, cost_t *v);
    int_t find_path_dense(
        const uint_t n, cost_t *cost[],
        const int_t start_i,
        int_t *y, cost_t *v,
        int_t *pred);
    int_t _ca_dense(
        const uint_t n, cost_t *cost[],
        const uint_t n_free_rows,
        int_t *free_rows, int_t *x, int_t *y, cost_t *v);


    #define INF std::numeric_limits<int>::max()
    #define VERBOSE false

    /* Pre-declare functions to allow arbitrary call ordering  */
    void auctionRound(std::vector<int>* assignment, std::vector<double>* prices, std::vector<double>* C, double epsilon);
    std::vector<int> getIndicesWithVal(std::vector<int>* v, int val);
    std::vector<int> makeRandC(int size);
    void reset(std::vector<int>* v, int val);
};



#endif
