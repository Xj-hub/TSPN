#ifndef TSP_RKGA_H
#define TSP_RKGA_H
#include "point.h"
#include <vector>
#include "gtsp/set.h"

struct Gene{
    int point_index;
    float fractional;
    int index;
    Gene(int point_index, float frac, int index):point_index(point_index), fractional(frac), index(index){};
    Gene():fractional(0.0), index(-1){};
};

struct Chromosome{
    float fitness;
    std::vector<Gene> dna;
    Chromosome():fitness(0.0),dna(std::vector<Gene>()){};
    Chromosome(int size):fitness(0.0),dna(std::vector<Gene>(size)){};
    Chromosome(float fitness):fitness(fitness),dna(std::vector<Gene>()){};
};

// ascending order comparator for gene fractional
inline bool compareAscendingFractional(const Gene &g1,const Gene &g2)
{
	return g1.fractional < g2.fractional;
};
// ascending order comparator for chromosome fitness
inline bool compareAscendingFitness(const Chromosome &c1,const Chromosome &c2)
{
	return c1.fitness < c2.fitness;
};


class RKGA
{
    std::vector<Set> sets;
    float Ps; // percentage of the new population constituted by selection
    float Px; // percentage of the new population constituted by crossover
    float Pu; // threshold for the crossover
    float Pm; // threshold for the mutation
    int ChromosomeSize; // define the size of each chromosome;
    int PopulaionSize;  // define the size of populaion;
    std::vector<Chromosome> population;// = std::vector<Chromosome>(PopulaionSize)
public:
    int MaxGeneration;  // define the max generation
    RKGA(
        std::vector<Set> sets,
        float Ps, // percentage of the new population constituted by selection
        float Px, // percentage of the new population constituted by crossover
        float Pu, // threshold for the crossover
        float Pm,
        // int ChromosomeSize, // define the size of each chromosome; 
        int PopulaionSize,  //define the size of populaion;
        int MaxGeneration   // define the max generation
    );
    void initialize();
    int select();
    int crossover();
    void mutate();
    void immigrate();
    void printResult();
    std::vector<std::pair<int,int>> calculatePath();
};
#endif