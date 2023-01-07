// Markus Buchholz
// g++ whale_algorithm_eng_function.cpp -o whale_eng -O3

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//-----------------------------------------------------------------------------

float xmin = 0.05;
float xmax = 2.0;
float ymin = 0.25;
float ymax = 1.3;
float zmin = 2.0;
float zmax = 15.0;

//--------------------------------------------------------------------------------
int DIM = 3;
int EVOLUTIONS = 5000;
int WHALES = 100;
float B = 1.0;
float R = 0.045;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
    float z;
};

//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

float generateRandomX()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);
    return distribution(gen);
}
//--------------------------------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {
        float p1{0}, p2{0}, p3{0}, p4{0};
        auto f1 = 1 - (float)(std::pow(ii.y, 2) * ii.z) / (float)(71785.0 * std::pow(ii.x, 4));

        auto f2 = (float)(4.0 * std::pow(ii.y, 2) - ii.x * ii.y) / (float)(12556.0 * (ii.y * std::pow(ii.x, 3) - std::pow(ii.x, 4))) + 1.0 / (float)(5108.0 * std::pow(ii.x, 2));

        auto f3 = 1.0 - (float)(140.45 * ii.x) / (float)(std::pow(ii.y, 2) * ii.z);

        auto f4 = ((float)(ii.x + ii.y) / (float)1.5) - 1.0;

        if (f1 > 0)
        {
            p1 = std::pow(f1, 2);
        }
        if (f2 > 0)
        {
            p2 = std::pow(f2, 2);
        }
        if (f3 > 0)
        {
            p3 = std::pow(f3, 2);
        }

        if (f4 > 0)
        {
            p4 = std::pow(f4, 2);
        }

        auto f = (ii.z + 2) * ii.y * std::pow(ii.x, 2) + R * p1 + R * p2 + R * p3 + R * p4;

        funcValue.push_back(f);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    if (Pnew.z < zmin)
    {
        Pnew.z = zmin;
    }

    if (Pnew.z > zmax)
    {
        Pnew.z = zmax;
    }

    return Pnew;
}

//--------------------------------------------------------------------------------

float vecModul(Pos vec)
{

    return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

//--------------------------------------------------------------------------------

Pos posNewWhale(Pos best, Pos whale, Pos rand, int iter)
{

    Pos Xnew, vecA, vecC, vec_a, vec_r;

    float p = generateRandom();

    vec_a.x = vec_a.y = vec_a.z = 2 - 2 * (float)iter / (float)EVOLUTIONS;
    vec_r.x = generateRandom();
    vec_r.y = generateRandom();
    vec_r.z = generateRandom();

    vecA.x = 2 * vec_a.x * vec_r.x - vec_a.x;
    vecA.y = 2 * vec_a.y * vec_r.y - vec_a.y;
    vecA.z = 2 * vec_a.z * vec_r.z - vec_a.z;

    vecC.x = 2 * vec_r.x;
    vecC.y = 2 * vec_r.y;
    vecC.z = 2 * vec_r.z;

    if (p < 0.5)
    {
        if (vecModul(vecA) < 1.0)
        {

            Xnew.x = best.x - vecA.x * std::abs(vecC.x * best.x - whale.x);
            Xnew.y = best.y - vecA.y * std::abs(vecC.y * best.y - whale.y);
            Xnew.z = best.z - vecA.z * std::abs(vecC.z * best.z - whale.z);
        }

        else if (vecModul(vecA) >= 1.0)
        {

            Xnew.x = rand.x - vecA.x * std::abs(vecC.x * rand.x - whale.x);
            Xnew.y = rand.y - vecA.y * std::abs(vecC.y * rand.y - whale.y);
            Xnew.z = rand.z - vecA.z * std::abs(vecC.z * rand.z - whale.z);
        }
    }
    else if (p >= 0.5)
    {

        float L = generateRandomX();

        Xnew.x = std::abs(best.x - whale.x) * std::exp(B * L) * std::cos(2 * M_PI * L) + best.x;
        Xnew.y = std::abs(best.y - whale.y) * std::exp(B * L) * std::cos(2 * M_PI * L) + best.y;
        Xnew.z = std::abs(best.z - whale.z) * std::exp(B * L) * std::cos(2 * M_PI * L) + best.z;
    }

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < WHALES; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax), valueGenerator(zmin, zmax)});
    }

    return pos;
}

//-------------------------------------------------------------------------------
bool compareMax(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second > b.second;
}

//-------------------------------------------------------------------------------

// max
std::tuple<Pos, float> findWorstPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMax);

    return best[0];
}

//-------------------------------------------------------------------------------
bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}

//-------------------------------------------------------------------------------

// min
std::tuple<Pos, float> findBestPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMin);

    return best[0];
}

//-------------------------------------------------------------------------------

int chooseRandomWhale(int actual)
{

    std::random_device engine;
    std::uniform_int_distribution<int> distribution(0, WHALES);

    int r = -1;

    do
    {

        r = distribution(engine);

    } while (r == actual);

    return r;
}

//-------------------------------------------------------------------------------

void runWAO()
{

    std::vector<Pos> currentPositions = initPosXY();
    std::vector<float> currentValueFunction = function(currentPositions);
    auto bestPosValueWhale = findBestPosFuncValue(currentPositions, currentValueFunction);

    for (int jj = 1; jj < EVOLUTIONS; jj++)
    {

        Pos bestPosWhale = std::get<0>(bestPosValueWhale);

        for (int ii = 0; ii < WHALES; ii++)
        {
            int randomWhale = chooseRandomWhale(ii);
            currentPositions[ii] = posNewWhale(bestPosWhale, currentPositions[ii], currentPositions[randomWhale], jj);
        }

        currentValueFunction = function(currentPositions);
        auto new_bestPosValueWhale = findBestPosFuncValue(currentPositions, currentValueFunction);
        if (std::get<1>(new_bestPosValueWhale) < std::get<1>(bestPosValueWhale))
        {
            bestPosValueWhale = new_bestPosValueWhale;
        }
    }
    // for (auto &ii : currentValueFunction)
    // {
    //     std::cout << ii << "\n";
    // }
    //   for (auto &ii : currentPositions){

    //       std::cout << ii.x << " ," << ii.y << " ," << ii.z << "\n";
    //   }

    bestPosValueWhale = findBestPosFuncValue(currentPositions, currentValueFunction);

    std::cout << "-------Optimization problem: tension/compression spring ----------\n";
    std::cout<< "min weight= " << std::get<1>(bestPosValueWhale)<< "\n";   
    std::cout<< "values   d= " <<std::get<0>(bestPosValueWhale).x << " D= " <<std::get<0>(bestPosValueWhale).y << " N= " <<std::get<0>(bestPosValueWhale).z << "\n";   
}

//-------------------------------------------------------------------------------

int main()
{

    runWAO();
}
