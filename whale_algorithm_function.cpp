// Markus Buchholz
#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//--------Path Planner--------------------------------------------------------------

float xmin = -5.0; // 0.0;
float xmax = 5.0;  // 50.0;
float ymin = -5.0; // 0.0;
float ymax = 5.0;  // 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//--------------------------------------------------------------------------------
int DIM = 2;
int EVOLUTIONS = 100;
int WHALES = 50;
float B = 1.0;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
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

        funcValue.push_back(ii.x * ii.y);
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

    return Pnew;
}

//--------------------------------------------------------------------------------

float vecModul(Pos vec)
{

    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

//--------------------------------------------------------------------------------

Pos posNewWhale(Pos best, Pos whale, Pos rand, int iter)
{

    Pos Xnew, vecA, vecC, vec_a, vec_r;

    float p = generateRandom();

    vec_a.x = vec_a.y = 2 - 2 * (float)iter / (float)EVOLUTIONS;
    vec_r.x = generateRandom();
    vec_r.y = generateRandom();

    vecA.x = 2 * vec_a.x * vec_r.x - vec_a.x;
    vecA.y = 2 * vec_a.y * vec_r.y - vec_a.y;

    vecC.x = 2 * vec_r.x;
    vecC.y = 2 * vec_r.y;

    if (p < 0.5)
    {
        if (vecModul(vecA) < 1.0)
        {

            Xnew.x = best.x - vecA.x * std::abs(vecC.x * best.x - whale.x);
            Xnew.y = best.y - vecA.y * std::abs(vecC.y * best.y - whale.y);
        }

        else if (vecModul(vecA) >= 1.0)
        {

            Xnew.x = rand.x - vecA.x * std::abs(vecC.x * rand.x - whale.x);
            Xnew.y = rand.y - vecA.y * std::abs(vecC.y * rand.y - whale.y);
        }
    }
    else if (p >= 0.5)
    {

        float L = generateRandomX();

        Xnew.x = std::abs(best.x - whale.x) * std::exp(B * L) * std::cos(2 * M_PI * L) + best.x;
        Xnew.y = std::abs(best.y - whale.y) * std::exp(B * L) * std::cos(2 * M_PI * L) + best.y;
    }

    return positionUpdateCheck(Xnew);
}


//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < WHALES; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
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

        Pos bestPosWhale = std::get<0> (bestPosValueWhale);

        for (int ii = 0; ii < WHALES; ii++)
        {
            int randomWhale = chooseRandomWhale(ii);
            currentPositions[ii] = posNewWhale(bestPosWhale, currentPositions[ii], currentPositions[randomWhale], jj);
        }

        currentValueFunction = function(currentPositions);
        auto new_bestPosValueWhale = findBestPosFuncValue(currentPositions, currentValueFunction);
        if(std::get<1>(new_bestPosValueWhale) < std::get<1>(bestPosValueWhale) ){
            bestPosValueWhale = new_bestPosValueWhale;
        }



    }
    for (auto &ii : currentValueFunction)
    {
        std::cout << ii << "\n";
    }
}

//-------------------------------------------------------------------------------

int main()
{

    runWAO();
}
