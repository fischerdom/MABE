

#ifndef __BasicMarkovBrainTemplate__WorldSwarm2__
#define __BasicMarkovBrainTemplate__WorldSwarm2__

#include "../AbstractWorld.h"

#include "../../Brain/MarkovBrain/MarkovBrain.h"

#include <stdlib.h>
#include <thread>
#include <vector>

using namespace std;

class Swarm2World : public AbstractWorld {
    
public:
    
    const int DIRECTIONS[8] = {1, 2, 3, 4, 5, 6, 7, 8}; // 1=e, 2=se, 3=s, 4=sw, 5=w, 6=nw, 7=n, 8=ne
    const int RELPOS[8][2] = {{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
    
    static shared_ptr<ParameterLink<int>> worldUpdatesPL;
    static shared_ptr<ParameterLink<int>> gridXSizePL;
    static shared_ptr<ParameterLink<int>> gridYSizePL;
    static shared_ptr<ParameterLink<int>> hasPenaltyPL;
    static shared_ptr<ParameterLink<int>> senseAgentsPL;
    static shared_ptr<ParameterLink<int>> detectWaterPL;
    static shared_ptr<ParameterLink<double>> nAgentsPL;
    static shared_ptr<ParameterLink<string>> senseSidesPL;
    static shared_ptr<ParameterLink<int>> resetOutputsPL;
    static shared_ptr<ParameterLink<double>> penaltyPL;
    static shared_ptr<ParameterLink<int>> waitForGoalPL;
    
    int** loadLevel();
    
    int generation;
    bool senseAgents;
    bool resetOutputs;
    bool hasPenalty;
    double nAgents;
    double penalty;
    int waitForGoalI;
    vector<int> senseSides;
    
    
    int gridX;
    int gridY;
    int **waterMap;
    int **bridgeMap;
    int **agentMap;
    int worldUpdates;
    
    pair<int,int> avgGoal;
    vector<pair<int,int>> startSlots;
    vector<pair<int,int>> location;
    vector<pair<int,int>> oldLocation;
    vector<double> score;
    vector<double> waitForGoal;
    vector<int> stack;
    
    Swarm2World(shared_ptr<ParametersTable> _PT = nullptr);
    virtual ~Swarm2World() = default;
    
    //virtual void evaluate(map<string, shared_ptr<Group>>& groups, int analyse = 0, int visualize = 0, int debug = 0)override;
    virtual void evaluateSolo(shared_ptr<Organism> org, int analyse, int visualize, int debug) override;
    
    virtual void buildGrid();
    
    virtual int requiredInputs() override;
    virtual int requiredOutputs() override;
    //virtual int maxOrgsAllowed();
    //virtual int minOrgsAllowed();
    
    int** zeros(int x, int y);
    void showMat(int** mat, int x, int y);
    void writeMap();
    
    bool isWater(pair<int,int> loc);
    bool inWater(pair<int,int> loc);
    bool isAgent(pair<int,int> loc);
    bool isValid(pair<int,int> loc);
    bool isFloor(pair<int,int> loc);
    //int countAgent(pair<int,int> loc, int group);
    bool isStart(pair<int,int> loc);
    bool isGoal(pair<int,int> loc);
    
    void move(int idx, pair<int,int> newloc);
    //bool canMove(pair<int,int> locB);
    
    
    //void placeAgent(int idx, pair<int,int> oldloc, pair<int,int> loc, int group, shared_ptr<Organism2D> org);
    pair<int,int> getRelativePosition(pair<int,int> loc, int facing, int direction);
    vector<pair<int,int>> getOuterPosition(pair<int,int> loc);
    vector<pair<int,int>> getInnerPosition(pair<int,int> loc);
    
    int ** getTPM(shared_ptr<MarkovBrain> brain);
    vector<vector<int>> getCM(shared_ptr<MarkovBrain> brain);
    
    // new in Swarm2
    
    double **pheroMap;
    
    double** zerosDouble(int x, int y);
    
    bool hasGoal(pair<int,int> loc);
    bool hasStart(pair<int,int> loc);
    bool hasAgent(pair<int,int> loc);
    bool hasPhero(pair<int,int> loc);
    bool hasDitch(pair<int,int> loc);
    bool isBridge(int idxs);
    
    void moveToGoal(int idx);
    void moveToStart(int idx);
    void moveToAgent(int idx);
    void moveToPhero(int idx);
    void moveToDitch(int idx);
    void moveToRandom(int idx);
    void decay();
    int distance(pair<int,int> a, pair<int,int> b);
    
};

#endif /* defined(__BasicMarkovBrainTemplate__WorldSwarm2__) */
