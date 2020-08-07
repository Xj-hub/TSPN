#include <iostream>
#include <time.h>    //reset random seed time(NULL)
#include <algorithm> //use sort
#include <assert.h>
#include <cfloat>    //define FLT_MAX
#include "gtsp/rkga.h"
#include "point.h"
#include "distance.h"
#include "util.h"


void calculateFitness(Chromosome & chromosome, std::vector<Set> &sets){
    //copy a new dna, sort it based on fractional, the path is gene.index
    std::vector<Gene> dna = chromosome.dna;
    std::sort(dna.begin(), dna.end(),compareAscendingFractional);
    int dna_size = dna.size();

    //push the start point to the path
    dna.push_back(dna.front());

    //calculate distance base on the distance function
    float cost = 0.0;
    for(int i = 0; i < dna_size; ++i){
        cost += euclideanDistance(sets[dna[i].index].points[dna[i].point_index], sets[dna[i+1].index].points[dna[i+1].point_index]);
    }
    chromosome.fitness = cost;
    return;
}

Chromosome generateRandomChomosome(int ChromosomeSize, std::vector<Set> &sets){
    Chromosome chromosome;
    for(int j = 0; j < ChromosomeSize; ++j){
        float frac = randomFloat(0.0,1.0);
        int point_index = randomInt(0, sets[j].points.size()-1);
        Gene gene(point_index, frac, j);
        chromosome.dna.push_back(gene);
        //calculate fitness for each chromosome
    }
    calculateFitness(chromosome, sets);
    return chromosome;
}

Chromosome selectParent(std::vector<Chromosome> &population, int selectionSize, int tournamentSize){
    assert(tournamentSize<selectionSize);
    Chromosome chromosome(FLT_MAX);
    for(int i = 0; i < tournamentSize; ++i){
        int random_int = randomInt(0, selectionSize-1);
        if (population[random_int].fitness < chromosome.fitness){
            chromosome = population[random_int];
        }
    }
    return chromosome;
}

RKGA::RKGA(
    std::vector<Set> sets,
    float Ps, // percentage of the new population constituted by selection
    float Px, // percentage of the new population constituted by crossover
    float Pu, // threshold for the crossover
    float Pm,
    // int ChromosomeSize, // define the size of each chromosome; 
    int PopulaionSize,  //define the size of populaion;
    int MaxGeneration
){
    this->sets = sets;
    this->Ps = Ps;
    this->Px = Px;
    this->Pu = Pu;
    this->Pm = Pm;
    this->PopulaionSize = PopulaionSize;
    this->ChromosomeSize = sets.size();
    this->MaxGeneration = MaxGeneration;
}

void RKGA::initialize(){
    for(int i = 0; i < PopulaionSize; ++i){
        Chromosome chromosome = generateRandomChomosome(ChromosomeSize, sets);
        population.push_back(chromosome);
    }
}

int RKGA::select(){
    int selection_size = (int)((float)PopulaionSize * Ps);

    //sort the chomosome in population in ascending order in population
    std::sort(population.begin(), population.end(), compareAscendingFitness);
    return selection_size;
}

int RKGA::crossover(){
    int crossover_size = (int)((float)PopulaionSize * Px);
    int selection_size = (int)((float)PopulaionSize * Ps);

    for(int i = 0; i < crossover_size; ++i){
        // first select two chromosome in selected pool using tournament porcedure
        // where the best among three randomly selected chromosomes in the pool 
        // becomes one parent
        Chromosome parent_1 = selectParent(population, selection_size, 3);
        Chromosome parent_2 = selectParent(population, selection_size, 3);

        // second generate a sequence of n random numbers in 0-1
        std::vector<float> random_sequence(ChromosomeSize);
        for(int i = 0; i < ChromosomeSize; ++i){
            random_sequence[i] = randomFloat(0.0, 1.0);
        }

        //select gene from parent_1 when random number < Pu
        Chromosome child(ChromosomeSize);
        for(int j = 0; j < ChromosomeSize; ++j){
            if(parent_1.dna[j].fractional <= Pu){
                child.dna[j] = parent_1.dna[j];
            }
            else{
                child.dna[j] = parent_2.dna[j];
            }
        }
        calculateFitness(child, sets);

        // std::cout<<parent_1.fitness<<' '<<parent_2.fitness<<' '<<child.fitness<<'\n';
        population[selection_size + i] = child;
    }
    return crossover_size;
}

void RKGA::mutate(){
    bool change = 0;
    for(Chromosome & ch: population){
        for(Gene & g: ch.dna){
            if(randomFloat(0.0,1.0) < Pm){
                g.fractional = randomFloat(0.0, 1.0);
                change = 1;
            }
            if(randomFloat(0.0,1.0) < Pm){
                g.point_index = randomInt(0, sets[g.index].points.size()-1);
                change = 1;
            }
        }
        if(change == 1){
            calculateFitness(ch, sets);
            change = 0;
        }
    }
}

void RKGA::immigrate(){
    int crossover_size = (int)((float)PopulaionSize * Px);
    int selection_size = (int)((float)PopulaionSize * Ps);
    int immigrate_size = PopulaionSize - crossover_size - selection_size;
    for(int i = 0; i < immigrate_size; ++i){
        population[crossover_size + selection_size + i] = generateRandomChomosome(ChromosomeSize, sets);
    }
}

void RKGA::printResult(){
    std::sort(population.begin(), population.end(), compareAscendingFitness);
    Chromosome best = population.front();
    std::sort(best.dna.begin(), best.dna.end(),compareAscendingFractional);
    for(Gene & gene: best.dna){
        std::cout<< gene.index<<' ';
    }
    std::cout<<'\n';
    std::cout<<population.front().fitness<<'\n';
}

std::vector<std::pair<int,int>> RKGA::calculatePath(){
    std::vector<std::pair<int, int>> path;
    std::sort(population.begin(), population.end(), compareAscendingFitness);
    Chromosome best = population.front();
    std::sort(best.dna.begin(), best.dna.end(),compareAscendingFractional);
    for(Gene & gene: best.dna){
        path.push_back({gene.index, gene.point_index});
    }
    return path;
}