#include "association.h"
#include <algorithm>
#include <string.h>



void Linear_Assigment::greedy_solver(std::vector<std::vector<cost_pair> > &cost_matrix,
                                     std::vector<int> &trace_assign_info,
                                     std::vector<int> &meas_assign_info)
{
    //获取所有航迹与所有航迹的“距离”
    std::vector<cost_pair> new_cost_mat;
    for(const auto &subList:cost_matrix)
    {
       for(auto sub_pair:subList)
       {
            new_cost_mat.push_back(sub_pair);
       }
    }

    //从小到大排列
    std::sort(new_cost_mat.begin(), new_cost_mat.end(), [](cost_pair c1, cost_pair c2)
    {
        return (c1.second < c2.second);
    });

    //遍历所有成员，进行分配
    for(const auto sub_pair:new_cost_mat)
    {
        if((trace_assign_info.at(sub_pair.first.first) == 255) && \
            (meas_assign_info.at(sub_pair.first.second) == 255))
        {
            trace_assign_info.at(sub_pair.first.first) = sub_pair.first.second;
            meas_assign_info.at(sub_pair.first.second) = sub_pair.first.first;
        }
    }

#if 0
    double total_cost = 0.0;
    for(uint idx=0; idx<trace_assign_info.size(); idx++)
    {
        if(trace_assign_info.at(idx) != 255)
        {
            total_cost += cost_matrix.at(idx).at(trace_assign_info.at(idx)).second;
        }
    }
    std::cout << "total is: " << std::to_string(total_cost) << std::endl;
#endif
}


//
// --------------------------------------------------------------------------
track_t Linear_Assigment::Hungarian_solver(const distMatrix_t& distMatrixIn,
                                       size_t nOfRows,
                                       size_t nOfColumns,
                                       std::vector<int>& assignment,
                                       HM_TMethod Method)
{
    assignment.resize(nOfRows, -1);

    track_t cost = 0;

    switch (Method)
    {
    case optimal:
        assignmentoptimal(assignment, cost, distMatrixIn, nOfRows, nOfColumns);
        break;

    case many_forbidden_assignments:
        assignmentsuboptimal1(assignment, cost, distMatrixIn, nOfRows, nOfColumns);
        break;

    case without_forbidden_assignments:
        assignmentsuboptimal2(assignment, cost, distMatrixIn, nOfRows, nOfColumns);
        break;
    }

    return cost;
}

// --------------------------------------------------------------------------
// Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
// --------------------------------------------------------------------------
void Linear_Assigment::assignmentoptimal(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns)
{
    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: Generate distance cv::Matrix and check cv::Matrix elements positiveness, assignment = " << assignment.size() << ", cost = " << cost << ", distMatrixIn = " << distMatrixIn.size() << ", nOfRows = " << nOfRows << ", nOfColumns = " << nOfColumns << std::endl;

    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: Total elements number" << std::endl;
    const size_t nOfElements = nOfRows * nOfColumns;
    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: Memory allocation" << std::endl;
    m_distMatrix.assign(std::begin(distMatrixIn), std::end(distMatrixIn));
    const track_t* distMatrixEnd = m_distMatrix.data() + nOfElements;

    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: Memory allocation" << std::endl;
    bool* coveredColumns = (bool*)calloc(nOfColumns, sizeof(bool));
    bool* coveredRows = (bool*)calloc(nOfRows, sizeof(bool));
    bool* starMatrix = (bool*)calloc(nOfElements, sizeof(bool));
    bool* primeMatrix = (bool*)calloc(nOfElements, sizeof(bool));
    bool* newStarMatrix = (bool*)calloc(nOfElements, sizeof(bool)); // used in step4

    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: preliminary steps" << std::endl;
    if (nOfRows <= nOfColumns)
    {
        for (size_t row = 0; row < nOfRows; ++row)
        {
            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentoptimal: find the smallest element in the row" << std::endl;
            track_t* distMatrixTemp = m_distMatrix.data() + row;
            track_t  minValue = *distMatrixTemp;
            distMatrixTemp += nOfRows;
            while (distMatrixTemp < distMatrixEnd)
            {
                track_t value = *distMatrixTemp;
                if (value < minValue)
                    minValue = value;

                distMatrixTemp += nOfRows;
            }
            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentoptimal: subtract the smallest element from each element of the row" << std::endl;
            distMatrixTemp = m_distMatrix.data() + row;
            while (distMatrixTemp < distMatrixEnd)
            {
                *distMatrixTemp -= minValue;
                distMatrixTemp += nOfRows;
            }
        }
        if(HUNGARIAN_LOGS)
                std::cout << "assignmentoptimal: Steps 1 and 2a" << std::endl;
        for (size_t row = 0; row < nOfRows; ++row)
        {
            for (size_t col = 0; col < nOfColumns; ++col)
            {
                if (m_distMatrix[row + nOfRows*col] == 0)
                {
                    if (!coveredColumns[col])
                    {
                        starMatrix[row + nOfRows * col] = true;
                        coveredColumns[col] = true;
                        break;
                    }
                }
            }
        }
    }
    else // if(nOfRows > nOfColumns)
    {
        for (size_t col = 0; col < nOfColumns; ++col)
        {
            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentoptimal: find the smallest element in the column" << std::endl;
            track_t* distMatrixTemp = m_distMatrix.data() + nOfRows*col;
            track_t* columnEnd = distMatrixTemp + nOfRows;
            track_t  minValue = *distMatrixTemp++;
            while (distMatrixTemp < columnEnd)
            {
                track_t value = *distMatrixTemp++;
                if (value < minValue)
                    minValue = value;
            }
            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentoptimal: subtract the smallest element from each element of the column" << std::endl;
            distMatrixTemp = m_distMatrix.data() + nOfRows*col;
            while (distMatrixTemp < columnEnd)
            {
                *distMatrixTemp++ -= minValue;
            }
        }
        if(HUNGARIAN_LOGS)
                std::cout << "assignmentoptimal: Steps 1 and 2a" << std::endl;
        for (size_t col = 0; col < nOfColumns; ++col)
        {
            for (size_t row = 0; row < nOfRows; ++row)
            {
                if (m_distMatrix[row + nOfRows*col] == 0)
                {
                    if (!coveredRows[row])
                    {
                        starMatrix[row + nOfRows*col] = true;
                        coveredColumns[col] = true;
                        coveredRows[row] = true;
                        break;
                    }
                }
            }
        }

        for (size_t row = 0; row < nOfRows; ++row)
        {
            coveredRows[row] = false;
        }
    }
    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: move to step 2b" << std::endl;
    step2b(assignment, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, (nOfRows <= nOfColumns) ? nOfRows : nOfColumns);
    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: compute cost and remove invalid assignments" << std::endl;
    computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);
    if(HUNGARIAN_LOGS)
            std::cout << "assignmentoptimal: free allocated memory" << std::endl;
    free(coveredColumns);
    free(coveredRows);
    free(starMatrix);
    free(primeMatrix);
    free(newStarMatrix);
    return;
}
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void Linear_Assigment::buildassignmentvector(assignments_t& assignment, bool *starMatrix, size_t nOfRows, size_t nOfColumns)
{
    for (size_t row = 0; row < nOfRows; ++row)
    {
        for (size_t col = 0; col < nOfColumns; ++col)
        {
            if (starMatrix[row + nOfRows * col])
            {
                assignment[row] = static_cast<int>(col);
                break;
            }
        }
    }
}
// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void Linear_Assigment::computeassignmentcost(const assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows)
{
    for (size_t row = 0; row < nOfRows; ++row)
    {
        const int col = assignment[row];
        if (col >= 0)
            cost += distMatrixIn[row + nOfRows * col];
    }
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void Linear_Assigment::step2a(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
    bool *starMatrixTemp, *columnEnd;
    if (HUNGARIAN_LOGS)
            std::cout << "step2a: cover every column containing a starred zero" << std::endl;
    for (size_t col = 0; col < nOfColumns; ++col)
    {
        starMatrixTemp = starMatrix + nOfRows * col;
        columnEnd = starMatrixTemp + nOfRows;
        while (starMatrixTemp < columnEnd)
        {
            if (*starMatrixTemp++)
            {
                coveredColumns[col] = true;
                break;
            }
        }
    }
    if(HUNGARIAN_LOGS)
            std::cout << "step2a: move to step 3" << std::endl;
    step2b(assignment, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void Linear_Assigment::step2b(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
    if(HUNGARIAN_LOGS)
            std::cout << "step2b: count covered columns" << std::endl;
    size_t nOfCoveredColumns = 0;
    for (size_t col = 0; col < nOfColumns; ++col)
    {
        if (coveredColumns[col])
            nOfCoveredColumns++;
    }
    if (nOfCoveredColumns == minDim) // algorithm finished
        buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
    else                             // move to step 3
        step3_5(assignment, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void Linear_Assigment::step3_5(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim)
{
    for (;;)
    {
        if(HUNGARIAN_LOGS)
                std::cout << "step3_5: step 3" << std::endl;
        bool zerosFound = true;
        while (zerosFound)
        {
            zerosFound = false;
            for (size_t col = 0; col < nOfColumns; ++col)
            {
                if (!coveredColumns[col])
                {
                    for (size_t row = 0; row < nOfRows; ++row)
                    {
                        if ((!coveredRows[row]) && (m_distMatrix[row + nOfRows*col] == 0))
                        {
                            if(HUNGARIAN_LOGS)
                                    std::cout << "step3_5: prime zero" << std::endl;
                            primeMatrix[row + nOfRows*col] = true;
                            if(HUNGARIAN_LOGS)
                                    std::cout << "step3_5: find starred zero in current row" << std::endl;
                            size_t starCol = 0;
                            for (; starCol < nOfColumns; ++starCol)
                            {
                                if (starMatrix[row + nOfRows * starCol])
                                    break;
                            }
                            if(HUNGARIAN_LOGS)
                                    std::cout << "step3_5: starCol = " << starCol << ", nOfColumns = " << nOfColumns << std::endl;
                            if (starCol == nOfColumns) // no starred zero found
                            {
                                if(HUNGARIAN_LOGS)
                                        std::cout << "step3_5: move to step 4" << std::endl;
                                step4(assignment, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
                                return;
                            }
                            else
                            {
                                coveredRows[row] = true;
                                coveredColumns[starCol] = false;
                                zerosFound = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
        if(HUNGARIAN_LOGS)
                std::cout << "step3_5: step 5" << std::endl;
        track_t h = std::numeric_limits<track_t>::max();
        for (size_t row = 0; row < nOfRows; ++row)
        {
            if (!coveredRows[row])
            {
                for (size_t col = 0; col < nOfColumns; ++col)
                {
                    if (!coveredColumns[col])
                    {
                        const track_t value = m_distMatrix[row + nOfRows*col];
                        if (value < h)
                            h = value;
                    }
                }
            }
        }
        if(HUNGARIAN_LOGS)
                std::cout << "step3_5: add h to each covered row, h = " << h << std::endl;
        for (size_t row = 0; row < nOfRows; ++row)
        {
            if (coveredRows[row])
            {
                for (size_t col = 0; col < nOfColumns; ++col)
                {
                    m_distMatrix[row + nOfRows*col] += h;
                }
            }
        }
        if(HUNGARIAN_LOGS)
                std::cout << "step3_5: subtract h from each uncovered column" << std::endl;
        for (size_t col = 0; col < nOfColumns; ++col)
        {
            if (!coveredColumns[col])
            {
                for (size_t row = 0; row < nOfRows; ++row)
                {
                    m_distMatrix[row + nOfRows*col] -= h;
                }
            }
        }
    }
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
void Linear_Assigment::step4(assignments_t& assignment, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col)
{
    const size_t nOfElements = nOfRows * nOfColumns;
    if(HUNGARIAN_LOGS)
            std::cout << "step4: generate temporary copy of starMatrix" << std::endl;
    for (size_t n = 0; n < nOfElements; ++n)
    {
        newStarMatrix[n] = starMatrix[n];
    }
    if(HUNGARIAN_LOGS)
            std::cout << "step4: star current zero" << std::endl;
    newStarMatrix[row + nOfRows*col] = true;
    if(HUNGARIAN_LOGS)
            std::cout << "step4: find starred zero in current column" << std::endl;
    size_t starCol = col;
    size_t starRow = 0;
    for (; starRow < nOfRows; ++starRow)
    {
        if (starMatrix[starRow + nOfRows * starCol])
            break;
    }
    while (starRow < nOfRows)
    {
        // unstar the starred zero
        newStarMatrix[starRow + nOfRows*starCol] = false;
        // find primed zero in current row
        size_t primeRow = starRow;
        size_t primeCol = 0;
        for (; primeCol < nOfColumns; ++primeCol)
        {
            if (primeMatrix[primeRow + nOfRows * primeCol])
                break;
        }
        // star the primed zero
        newStarMatrix[primeRow + nOfRows*primeCol] = true;
        // find starred zero in current column
        starCol = primeCol;
        for (starRow = 0; starRow < nOfRows; ++starRow)
        {
            if (starMatrix[starRow + nOfRows * starCol])
                break;
        }
    }
    // use temporary copy as new starMatrix
    // delete all primes, uncover all rows
    for (size_t n = 0; n < nOfElements; ++n)
    {
        primeMatrix[n] = false;
        starMatrix[n] = newStarMatrix[n];
    }
    for (size_t n = 0; n < nOfRows; ++n)
    {
        coveredRows[n] = false;
    }
    if(HUNGARIAN_LOGS)
            std::cout << "move to step 2a" << std::endl;
    step2a(assignment, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

// --------------------------------------------------------------------------
// Computes a suboptimal solution. Good for cases without forbidden assignments.
// --------------------------------------------------------------------------
void Linear_Assigment::assignmentsuboptimal2(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns)
{
    if(HUNGARIAN_LOGS)
            std::cout << "make working copy of distance Matrix" << std::endl;
    m_distMatrix.assign(std::begin(distMatrixIn), std::end(distMatrixIn));

    if(HUNGARIAN_LOGS)
            std::cout << "recursively search for the minimum element and do the assignment" << std::endl;
    for (;;)
    {
        // find minimum distance observation-to-track pair
        track_t minValue = std::numeric_limits<track_t>::max();
        size_t tmpRow = 0;
        size_t tmpCol = 0;
        for (size_t row = 0; row < nOfRows; ++row)
        {
            for (size_t col = 0; col < nOfColumns; ++col)
            {
                const track_t value = m_distMatrix[row + nOfRows*col];
                if (value != std::numeric_limits<track_t>::max() && (value < minValue))
                {
                    minValue = value;
                    tmpRow = row;
                    tmpCol = col;
                }
            }
        }

        if (minValue != std::numeric_limits<track_t>::max())
        {
            assignment[tmpRow] = static_cast<int>(tmpCol);
            cost += minValue;
            for (size_t n = 0; n < nOfRows; ++n)
            {
                m_distMatrix[n + nOfRows*tmpCol] = std::numeric_limits<track_t>::max();
            }
            for (size_t n = 0; n < nOfColumns; ++n)
            {
                m_distMatrix[tmpRow + nOfRows*n] = std::numeric_limits<track_t>::max();
            }
        }
        else
        {
            break;
        }
    }
}
// --------------------------------------------------------------------------
// Computes a suboptimal solution. Good for cases with many forbidden assignments.
// --------------------------------------------------------------------------
void Linear_Assigment::assignmentsuboptimal1(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns)
{
    if(HUNGARIAN_LOGS)
            std::cout << "assignmentsuboptimal1: make working copy of distance Matrix" << std::endl;
    m_distMatrix.assign(std::begin(distMatrixIn), std::end(distMatrixIn));

    if(HUNGARIAN_LOGS)
            std::cout << "assignmentsuboptimal1: allocate memory" << std::endl;
    int* nOfValidObservations = (int *)calloc(nOfRows, sizeof(int));
    int* nOfValidTracks = (int *)calloc(nOfColumns, sizeof(int));

    if(HUNGARIAN_LOGS)
            std::cout << "assignmentsuboptimal1: compute number of validations" << std::endl;
    bool infiniteValueFound = false;
    bool finiteValueFound = false;
    for (size_t row = 0; row < nOfRows; ++row)
    {
        for (size_t col = 0; col < nOfColumns; ++col)
        {
            if (m_distMatrix[row + nOfRows*col] != std::numeric_limits<track_t>::max())
            {
                nOfValidTracks[col] += 1;
                nOfValidObservations[row] += 1;
                finiteValueFound = true;
            }
            else
            {
                infiniteValueFound = true;
            }
        }
    }

    if (infiniteValueFound)
    {
        if (!finiteValueFound)
        {
            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentsuboptimal1: free allocated memory" << std::endl;
            free(nOfValidObservations);
            free(nOfValidTracks);
            return;
        }
        bool repeatSteps = true;

        while (repeatSteps)
        {
            repeatSteps = false;

            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentsuboptimal1: step 1: reject assignments of multiply validated tracks to singly validated observation" << std::endl;
            for (size_t col = 0; col < nOfColumns; ++col)
            {
                bool singleValidationFound = false;
                for (size_t row = 0; row < nOfRows; ++row)
                {
                    if (m_distMatrix[row + nOfRows * col] != std::numeric_limits<track_t>::max() && (nOfValidObservations[row] == 1))
                    {
                        singleValidationFound = true;
                        break;
                    }
                }
                if (singleValidationFound)
                {
                    for (size_t nestedRow = 0; nestedRow < nOfRows; ++nestedRow)
                        if ((nOfValidObservations[nestedRow] > 1) && m_distMatrix[nestedRow + nOfRows * col] != std::numeric_limits<track_t>::max())
                        {
                            m_distMatrix[nestedRow + nOfRows * col] = std::numeric_limits<track_t>::max();
                            nOfValidObservations[nestedRow] -= 1;
                            nOfValidTracks[col] -= 1;
                            repeatSteps = true;
                        }
                }
            }

            if(HUNGARIAN_LOGS)
                    std::cout << "assignmentsuboptimal1: step 2: reject assignments of multiply validated observations to singly validated tracks" << std::endl;
            if (nOfColumns > 1)
            {
                for (size_t row = 0; row < nOfRows; ++row)
                {
                    bool singleValidationFound = false;
                    for (size_t col = 0; col < nOfColumns; ++col)
                    {
                        if (m_distMatrix[row + nOfRows*col] != std::numeric_limits<track_t>::max() && (nOfValidTracks[col] == 1))
                        {
                            singleValidationFound = true;
                            break;
                        }
                    }

                    if (singleValidationFound)
                    {
                        for (size_t col = 0; col < nOfColumns; ++col)
                        {
                            if ((nOfValidTracks[col] > 1) && m_distMatrix[row + nOfRows*col] != std::numeric_limits<track_t>::max())
                            {
                                m_distMatrix[row + nOfRows*col] = std::numeric_limits<track_t>::max();
                                nOfValidObservations[row] -= 1;
                                nOfValidTracks[col] -= 1;
                                repeatSteps = true;
                            }
                        }
                    }
                }
            }
        } // while(repeatSteps)

        if(HUNGARIAN_LOGS)
                std::cout << "assignmentsuboptimal1: for each multiply validated track that validates only with singly validated observations, choose the observation with minimum distance" << std::endl;
        for (size_t row = 0; row < nOfRows; ++row)
        {
            if (nOfValidObservations[row] > 1)
            {
                bool allSinglyValidated = true;
                track_t minValue = std::numeric_limits<track_t>::max();
                size_t tmpCol = 0;
                for (size_t col = 0; col < nOfColumns; ++col)
                {
                    const track_t value = m_distMatrix[row + nOfRows*col];
                    if (value != std::numeric_limits<track_t>::max())
                    {
                        if (nOfValidTracks[col] > 1)
                        {
                            allSinglyValidated = false;
                            break;
                        }
                        else if ((nOfValidTracks[col] == 1) && (value < minValue))
                        {
                            tmpCol = col;
                            minValue = value;
                        }
                    }
                }

                if (allSinglyValidated)
                {
                    assignment[row] = static_cast<int>(tmpCol);
                    cost += minValue;
                    for (size_t n = 0; n < nOfRows; ++n)
                    {
                        m_distMatrix[n + nOfRows*tmpCol] = std::numeric_limits<track_t>::max();
                    }
                    for (size_t n = 0; n < nOfColumns; ++n)
                    {
                        m_distMatrix[row + nOfRows*n] = std::numeric_limits<track_t>::max();
                    }
                }
            }
        }

        if(HUNGARIAN_LOGS)
                std::cout << "assignmentsuboptimal1: for each multiply validated observation that validates only with singly validated  track, choose the track with minimum distance" << std::endl;
        for (size_t col = 0; col < nOfColumns; ++col)
        {
            if (nOfValidTracks[col] > 1)
            {
                bool allSinglyValidated = true;
                track_t minValue = std::numeric_limits<track_t>::max();
                size_t tmpRow = 0;
                for (size_t row = 0; row < nOfRows; ++row)
                {
                    const track_t value = m_distMatrix[row + nOfRows*col];
                    if (value != std::numeric_limits<track_t>::max())
                    {
                        if (nOfValidObservations[row] > 1)
                        {
                            allSinglyValidated = false;
                            break;
                        }
                        else if ((nOfValidObservations[row] == 1) && (value < minValue))
                        {
                            tmpRow = row;
                            minValue = value;
                        }
                    }
                }

                if (allSinglyValidated)
                {
                    assignment[tmpRow] = static_cast<int>(col);
                    cost += minValue;
                    for (size_t n = 0; n < nOfRows; ++n)
                    {
                        m_distMatrix[n + nOfRows*col] = std::numeric_limits<track_t>::max();
                    }
                    for (size_t n = 0; n < nOfColumns; ++n)
                    {
                        m_distMatrix[tmpRow + nOfRows*n] = std::numeric_limits<track_t>::max();
                    }
                }
            }
        }
    } // if(infiniteValueFound)


    if(HUNGARIAN_LOGS)
            std::cout << "assignmentsuboptimal1: now, recursively search for the minimum element and do the assignment" << std::endl;
    for (;;)
    {
        // find minimum distance observation-to-track pair
        track_t minValue = std::numeric_limits<track_t>::max();
        size_t tmpRow = 0;
        size_t tmpCol = 0;
        for (size_t row = 0; row < nOfRows; ++row)
        {
            for (size_t col = 0; col < nOfColumns; ++col)
            {
                const track_t value = m_distMatrix[row + nOfRows*col];
                if (value != std::numeric_limits<track_t>::max() && (value < minValue))
                {
                    minValue = value;
                    tmpRow = row;
                    tmpCol = col;
                }
            }
        }

        if (minValue != std::numeric_limits<track_t>::max())
        {
            assignment[tmpRow] = static_cast<int>(tmpCol);
            cost += minValue;
            for (size_t n = 0; n < nOfRows; ++n)
            {
                m_distMatrix[n + nOfRows*tmpCol] = std::numeric_limits<track_t>::max();
            }
            for (size_t n = 0; n < nOfColumns; ++n)
            {
                m_distMatrix[tmpRow + nOfRows*n] = std::numeric_limits<track_t>::max();
            }
        }
        else
        {
            break;
        }
    }

    if(HUNGARIAN_LOGS)
            std::cout << "assignmentsuboptimal1: free allocated memory" << std::endl;
    free(nOfValidObservations);
    free(nOfValidTracks);
}





int_t Linear_Assigment::_ccrrt_dense(const uint_t n, cost_t *cost[],
                    int_t *free_rows, int_t *x, int_t *y, cost_t *v)
{
    int_t n_free_rows;
    boolean *unique;

    for (uint_t i = 0; i < n; i++)
    {
        x[i] = -1;
        v[i] = LARGE;
        y[i] = 0;
    }
    for (uint_t i = 0; i < n; i++)
    {
        for (uint_t j = 0; j < n; j++)
        {
            const cost_t c = cost[i][j];
            if (c < v[j])
            {
                v[j] = c;
                y[j] = i;
            }
            PRINTF("i=%d, j=%d, c[i,j]=%f, v[j]=%f y[j]=%d\n", i, j, c, v[j], y[j]);
        }
    }
    PRINT_COST_ARRAY(v, n);
    PRINT_INDEX_ARRAY(y, n);
    NEW(unique, boolean, n);
    memset(unique, TRUE, n);
    {
        int_t j = n;
        do
        {
            j--;
            const int_t i = y[j];
            if (x[i] < 0)
            {
                x[i] = j;
            }
            else
            {
                unique[i] = FALSE;
                y[j] = -1;
            }
        } while (j > 0);
    }
    n_free_rows = 0;
    for (uint_t i = 0; i < n; i++)
    {
        if (x[i] < 0)
        {
            free_rows[n_free_rows++] = i;
        }
        else if (unique[i])
        {
            const int_t j = x[i];
            cost_t min = LARGE;
            for (uint_t j2 = 0; j2 < n; j2++)
            {
                if (j2 == (uint_t)j)
                {
                    continue;
                }
                const cost_t c = cost[i][j2] - v[j2];
                if (c < min)
                {
                    min = c;
                }
            }
            PRINTF("v[%d] = %f - %f\n", j, v[j], min);
            v[j] -= min;
        }
    }
    FREE(unique);
    return n_free_rows;
}

/** Augmenting row reduction for a dense cost matrix.
 */
int_t Linear_Assigment::_carr_dense(
    const uint_t n, cost_t *cost[],
    const uint_t n_free_rows,
    int_t *free_rows, int_t *x, int_t *y, cost_t *v)
{
    uint_t current = 0;
    int_t new_free_rows = 0;
    uint_t rr_cnt = 0;
    PRINT_INDEX_ARRAY(x, n);
    PRINT_INDEX_ARRAY(y, n);
    PRINT_COST_ARRAY(v, n);
    PRINT_INDEX_ARRAY(free_rows, n_free_rows);
    while (current < n_free_rows)
    {
        int_t i0;
        int_t j1, j2;
        cost_t v1, v2, v1_new;
        boolean v1_lowers;

        rr_cnt++;
        PRINTF("current = %d rr_cnt = %d\n", current, rr_cnt);
        const int_t free_i = free_rows[current++];
        j1 = 0;
        v1 = cost[free_i][0] - v[0];
        j2 = -1;
        v2 = LARGE;
        for (uint_t j = 1; j < n; j++)
        {
            PRINTF("%d = %f %d = %f\n", j1, v1, j2, v2);
            const cost_t c = cost[free_i][j] - v[j];
            if (c < v2)
            {
                if (c >= v1)
                {
                    v2 = c;
                    j2 = j;
                }
                else
                {
                    v2 = v1;
                    v1 = c;
                    j2 = j1;
                    j1 = j;
                }
            }
        }
        i0 = y[j1];
        v1_new = v[j1] - (v2 - v1);
        v1_lowers = v1_new < v[j1];
        PRINTF("%d %d 1=%d,%f 2=%d,%f v1'=%f(%d,%g) \n", free_i, i0, j1, v1, j2, v2, v1_new, v1_lowers, v[j1] - v1_new);
        if (rr_cnt < current * n)
        {
            if (v1_lowers)
            {
                v[j1] = v1_new;
            }
            else if (i0 >= 0 && j2 >= 0)
            {
                j1 = j2;
                i0 = y[j2];
            }
            if (i0 >= 0)
            {
                if (v1_lowers)
                {
                    free_rows[--current] = i0;
                }
                else
                {
                    free_rows[new_free_rows++] = i0;
                }
            }
        }
        else
        {
            PRINTF("rr_cnt=%d >= %d (current=%d * n=%d)\n", rr_cnt, current * n, current, n);
            if (i0 >= 0)
            {
                free_rows[new_free_rows++] = i0;
            }
        }
        x[free_i] = j1;
        y[j1] = free_i;
    }
    return new_free_rows;
}

/** Find columns with minimum d[j] and put them on the SCAN list.
 */
uint_t Linear_Assigment::_find_dense(const uint_t n, uint_t lo, cost_t *d, int_t *cols, int_t *y)
{
    uint_t hi = lo + 1;
    cost_t mind = d[cols[lo]];
    for (uint_t k = hi; k < n; k++)
    {
        int_t j = cols[k];
        if (d[j] <= mind)
        {
            if (d[j] < mind)
            {
                hi = lo;
                mind = d[j];
            }
            cols[k] = cols[hi];
            cols[hi++] = j;
        }
    }
    return hi;
}

// Scan all columns in TODO starting from arbitrary column in SCAN
// and try to decrease d of the TODO columns using the SCAN column.
int_t Linear_Assigment::_scan_dense(const uint_t n, cost_t *cost[],
                  uint_t *plo, uint_t *phi,
                  cost_t *d, int_t *cols, int_t *pred,
                  int_t *y, cost_t *v)
{
    uint_t lo = *plo;
    uint_t hi = *phi;
    cost_t h, cred_ij;

    while (lo != hi)
    {
        int_t j = cols[lo++];
        const int_t i = y[j];
        const cost_t mind = d[j];
        h = cost[i][j] - v[j] - mind;
        PRINTF("i=%d j=%d h=%f\n", i, j, h);
        // For all columns in TODO
        for (uint_t k = hi; k < n; k++)
        {
            j = cols[k];
            cred_ij = cost[i][j] - v[j] - h;
            if (cred_ij < d[j])
            {
                d[j] = cred_ij;
                pred[j] = i;
                if (cred_ij == mind)
                {
                    if (y[j] < 0)
                    {
                        return j;
                    }
                    cols[k] = cols[hi];
                    cols[hi++] = j;
                }
            }
        }
    }
    *plo = lo;
    *phi = hi;
    return -1;
}

/** Single iteration of modified Dijkstra shortest path algorithm as explained in the JV paper.
 *
 * This is a dense matrix version.
 *
 * \return The closest free column index.
 */
int_t Linear_Assigment::find_path_dense(
    const uint_t n, cost_t *cost[],
    const int_t start_i,
    int_t *y, cost_t *v,
    int_t *pred)
{
    uint_t lo = 0, hi = 0;
    int_t final_j = -1;
    uint_t n_ready = 0;
    int_t *cols;
    cost_t *d;

    NEW(cols, int_t, n);
    NEW(d, cost_t, n);

    for (uint_t i = 0; i < n; i++)
    {
        cols[i] = i;
        pred[i] = start_i;
        d[i] = cost[start_i][i] - v[i];
    }
    PRINT_COST_ARRAY(d, n);
    while (final_j == -1)
    {
        // No columns left on the SCAN list.
        if (lo == hi)
        {
            PRINTF("%d..%d -> find\n", lo, hi);
            n_ready = lo;
            hi = _find_dense(n, lo, d, cols, y);
            PRINTF("check %d..%d\n", lo, hi);
            PRINT_INDEX_ARRAY(cols, n);
            for (uint_t k = lo; k < hi; k++)
            {
                const int_t j = cols[k];
                if (y[j] < 0)
                {
                    final_j = j;
                }
            }
        }
        if (final_j == -1)
        {
            PRINTF("%d..%d -> scan\n", lo, hi);
            final_j = _scan_dense(
                n, cost, &lo, &hi, d, cols, pred, y, v);
            PRINT_COST_ARRAY(d, n);
            PRINT_INDEX_ARRAY(cols, n);
            PRINT_INDEX_ARRAY(pred, n);
        }
    }

    PRINTF("found final_j=%d\n", final_j);
    PRINT_INDEX_ARRAY(cols, n);
    {
        const cost_t mind = d[cols[lo]];
        for (uint_t k = 0; k < n_ready; k++)
        {
            const int_t j = cols[k];
            v[j] += d[j] - mind;
        }
    }

    FREE(cols);
    FREE(d);

    return final_j;
}

/** Augment for a dense cost matrix.
 */
int_t Linear_Assigment::_ca_dense(
    const uint_t n, cost_t *cost[],
    const uint_t n_free_rows,
    int_t *free_rows, int_t *x, int_t *y, cost_t *v)
{
    int_t *pred;

    NEW(pred, int_t, n);

    for (int_t *pfree_i = free_rows; pfree_i < free_rows + n_free_rows; pfree_i++)
    {
        int_t i = -1, j;
        uint_t k = 0;

        PRINTF("looking at free_i=%d\n", *pfree_i);
        j = find_path_dense(n, cost, *pfree_i, y, v, pred);
        ASSERT(j >= 0);
        ASSERT(j < n);
        while (i != *pfree_i)
        {
            PRINTF("augment %d\n", j);
            PRINT_INDEX_ARRAY(pred, n);
            i = pred[j];
            PRINTF("y[%d]=%d -> %d\n", j, y[j], i);
            y[j] = i;
            PRINT_INDEX_ARRAY(x, n);
            SWAP_INDICES(j, x[i]);
            k++;
            if (k >= n)
            {
                ASSERT(FALSE);
            }
        }
    }
    FREE(pred);
    return 0;
}

/** Solve dense sparse LAP.
 */
int Linear_Assigment::lapjv_internal(const uint_t n, cost_t *cost[],
    int_t *x, int_t *y)
{
    int ret;
    int_t *free_rows;
    cost_t *v;

    NEW(free_rows, int_t, n);
    NEW(v, cost_t, n);
    ret = _ccrrt_dense(n, cost, free_rows, x, y, v);
    int i = 0;
    while (ret > 0 && i < 2)
    {
        ret = _carr_dense(n, cost, ret, free_rows, x, y, v);
        i++;
    }
    if (ret > 0)
    {
        ret = _ca_dense(n, cost, ret, free_rows, x, y, v);
    }
    FREE(v);
    FREE(free_rows);
    return ret;
}



void Linear_Assigment::auction(int N, double **cost_c, std::vector<int> &assignment)
{
    std::vector<double> C(N * N);

    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j < N; j++)
        {
            C[i * N + j] = cost_c[i][j];//rand() % size + 1;
        }
    }

    assignment.resize(N);
    std::vector<double> prices(N, 1);
    double epsilon = 1;
    int iter = 1;

    while(epsilon > 1.0/N)
    {
        reset(&assignment, INF);
        while (find(assignment.begin(), assignment.end(), INF) != assignment.end())
        {
            iter++;
            auctionRound(&assignment, &prices, &C, epsilon);
        }
        epsilon = epsilon * .05;
    }

     std::cout << "Num Iterations:\t" << iter << std::endl;
     std::cout << "Total CPU time:\t" << time << std::endl;
     std::cout << std::endl << std::endl << "Solution: "  << std::endl;
     for (int i = 0; i < assignment.size(); i++)
     {
        std::cout << "Person " << i << " gets object " << assignment[i] << " " << prices[i] << std::endl;
     }
}

void Linear_Assigment::auctionRound(std::vector<int>* assignment, std::vector<double>* prices, std::vector<double>* C, double epsilon)
{
    int N = prices->size();

    /*
        These are meant to be kept in correspondance such that bidded[i]
        and bids[i] correspond to person i bidding for bidded[i] with bid bids[i]
    */
    std::vector<int> tmpBidded;
    std::vector<double> tmpBids;
    std::vector<int> unAssig;

    /* Compute the bids of each unassigned individual and store them in temp */
    for (int i = 0; i < assignment->size(); i++)
    {
        if (assignment->at(i) == INF)
        {
            unAssig.push_back(i);

            /*
                Need the best and second best value of each object to this person
                where value is calculated row_{j} - prices{j}
            */
            double optValForI = -INF;
            double secOptValForI = -INF;
            int optObjForI, secOptObjForI;
            for (int j = 0; j < N; j++)
            {
                double curVal = C->at(j + i*N) - prices->at(j);
                if (curVal > optValForI)
                {
                    secOptValForI = optValForI;
                    secOptObjForI = optObjForI;
                    optValForI = curVal;
                    optObjForI = j;
                }
                else if (curVal > secOptValForI)
                {
                    secOptValForI = curVal;
                    secOptObjForI = j;
                }
            }

            /* Computes the highest reasonable bid for the best object for this person */
            double bidForI = optValForI - secOptValForI + epsilon;

            /* Stores the bidding info for future use */
            tmpBidded.push_back(optObjForI);
            tmpBids.push_back(bidForI);
        }
    }

    /*
        Each object which has received a bid determines the highest bidder and
        updates its price accordingly
    */
    for (int j = 0; j < N; j++)
    {
        std::vector<int> indices = getIndicesWithVal(&tmpBidded, j);
        if (indices.size() != 0)
        {
            /* Need the highest bid for object j */
            double highestBidForJ = -INF;
            int i_j;
            for (int i = 0; i < indices.size(); i++)
            {
                double curVal = tmpBids.at(indices.at(i));
                if (curVal > highestBidForJ)
                {
                    highestBidForJ = curVal;
                    i_j = indices.at(i);
                }
            }

            /* Find the other person who has object j and make them unassigned */
            for (int i = 0; i < assignment->size(); i++)
            {
                if (assignment->at(i) == j)
                {
                    assignment->at(i) = INF;
                    break;
                }
            }

            /* Assign object j to i_j and update the price vector */
            assignment->at(unAssig[i_j]) = j;
            prices->at(j) = prices->at(j) + highestBidForJ;
        }
    }
}


/*<--------------------------------------   Utility Functions   -------------------------------------->*/

std::vector<int> Linear_Assigment::makeRandC(int size)
{
    srand (time(NULL));
    std::vector<int> mat(size * size, 2);
    for(int i = 0; i < size; i++)
    {
        for(int j = 0; j < size; j++)
        {
            mat[i + j * size] = rand() % size + 1;
        }
    }
    return mat;
}

/* Returns a vector of indices from v which have the specified value val */
std::vector<int> Linear_Assigment::getIndicesWithVal(std::vector<int>* v, int val)
{
    std::vector<int> out;
    for (int i = 0; i < v->size(); i++)
    {
        if (v->at(i) == val)
        {
            out.push_back(i);
        }
    }
    return out;
}

void Linear_Assigment::reset(std::vector<int>* v, int val)
{
    for (int i = 0; i < v->size(); i++)
    {
        v->at(i) = val;
    }
}
