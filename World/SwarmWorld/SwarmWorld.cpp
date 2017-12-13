
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>

#include "SwarmWorld.h"
#include "../../Organism/Organism.h"

shared_ptr<ParameterLink<int>> SwarmWorld::gridXSizePL = Parameters::register_parameter("WORLD_SWARM-gridX", 16, "size of grid X");
shared_ptr<ParameterLink<int>> SwarmWorld::gridYSizePL = Parameters::register_parameter("WORLD_SWARM-gridY", 10, "size of grid Y");
shared_ptr<ParameterLink<int>> SwarmWorld::worldUpdatesPL = Parameters::register_parameter("WORLD_SWARM-worldUpdates", 100, "amount of time a brain is tested");
shared_ptr<ParameterLink<double>> SwarmWorld::nAgentsPL = Parameters::register_parameter("WORLD_SWARM-nAgents", 1.0, "how many agents in a game in rate");
shared_ptr<ParameterLink<int>> SwarmWorld::senseAgentsPL = Parameters::register_parameter("WORLD_SWARM-senseAgents", 0, "1 if ants can sense");
shared_ptr<ParameterLink<string>> SwarmWorld::senseSidesPL = Parameters::register_parameter("WORLD_SWARM-senseSides", (string)"[1]", "1 if ants can sense");
shared_ptr<ParameterLink<int>> SwarmWorld::resetOutputsPL = Parameters::register_parameter("WORLD_SWARM-resetOutputs", 0, "1 if outputs should be reseted after one time step");
shared_ptr<ParameterLink<int>> SwarmWorld::pheroPL = Parameters::register_parameter("WORLD_SWARM-phero", 0, "do it with pheromones sexy");
shared_ptr<ParameterLink<int>> SwarmWorld::blockWayPL = Parameters::register_parameter("WORLD_SWARM-blockWay", 0, "1 if overlap is disabled");
shared_ptr<ParameterLink<int>> SwarmWorld::hasPenaltyPL = Parameters::register_parameter("WORLD_SWARM-hasPenalty", 1, "1 if penalty when agents get hit");
shared_ptr<ParameterLink<double>> SwarmWorld::penaltyPL = Parameters::register_parameter("WORLD_SWARM-penalty", 0.075, "amount of penalty for hit");
shared_ptr<ParameterLink<int>> SwarmWorld::waitForGoalPL = Parameters::register_parameter("WORLD_SWARM-waitForGoal", 500, "timestep till the next goal is possible");
shared_ptr<ParameterLink<int>> SwarmWorld::hiddenAgentsPL = Parameters::register_parameter("WORLD_SWARM-hiddenAgents", 0, "sensor without response");

SwarmWorld::SwarmWorld(shared_ptr<ParametersTable> _PT) : AbstractWorld(_PT) {
    cout << "Using SwarmWorld \n";
    
    worldUpdates = (PT == nullptr) ? worldUpdatesPL->lookup() : PT->lookupInt("WORLD_SWARM-worldUpdates");
    gridX = (PT == nullptr) ? gridXSizePL->lookup() : PT->lookupInt("WORLD_SWARM-gridX");
    gridY = (PT == nullptr) ? gridYSizePL->lookup() : PT->lookupInt("WORLD_SWARM-gridY");
    senseAgents = ((PT == nullptr) ? senseAgentsPL->lookup() : PT->lookupInt("WORLD_SWARM-senseAgents")) == 1;
    resetOutputs = ((PT == nullptr) ? resetOutputsPL->lookup() : PT->lookupInt("WORLD_SWARM-resetOutputs")) == 1;
    blockWay = ((PT == nullptr) ? blockWayPL->lookup() : PT->lookupInt("WORLD_SWARM-blockWay")) == 1;
    hasPenalty = ((PT == nullptr) ? hasPenaltyPL->lookup() : PT->lookupInt("WORLD_SWARM-hasPenalty")) == 1;
    nAgents = ((PT == nullptr) ? nAgentsPL->lookup() : PT->lookupDouble("WORLD_SWARM-nAgents"));
    convertCSVListToVector(((PT == nullptr) ? senseSidesPL->lookup() : PT->lookupString("WORLD_SWARM-senseSides")), senseSides);
    
    
    penalty = (PT == nullptr) ? penaltyPL->lookup() : PT->lookupDouble("WORLD_SWARM-penalty");
    phero = ((PT == nullptr) ? senseAgentsPL->lookup() : PT->lookupInt("WORLD_SWARM-phero")) == 1;
    waitForGoalI = (PT == nullptr) ? waitForGoalPL->lookup() : PT->lookupInt("WORLD_SWARM-waitForGoal");
    hiddenAgents = (PT == nullptr) ? hiddenAgentsPL->lookup() : PT->lookupInt("WORLD_SWARM-hiddenAgents") == 1;
    
    generation = 0;
    
    cout << worldUpdates << " Updates\n";
    cout << gridX << " X\n";
    cout << gridY << " Y\n";
    cout << senseAgents << " Sensor Agents\n";
    cout << hiddenAgents << " Hidden Agents\n";
    cout << resetOutputs << " Reset Outputs\n";
    cout << hasPenalty << " Penalty set\n";
    cout << nAgents << " factor agents\n";
    cout << PT->lookupString("WORLD_SWARM-senseSides") << " SenseSides\n";
    cout << penalty << " Penalty\n";
    cout << waitForGoalI << " Waitforgoal\n";
    cout << blockWay << " Block Way\n";
    cout << phero << " Phero\n";
    
    // columns to be added to ave file
    aveFileColumns.clear();
    aveFileColumns.push_back("score");
    std::remove("positions.csv");
    
    this->buildGrid();
    
}

void SwarmWorld::buildGrid(){
    
    cout << "Build Map:\n";
    this->waterMap = loadLevel();
    
    
    for(int i = 0; i < gridY; i++) {
        for(int j = 0; j < gridX; j++) {
            pair<int,int> loc = {j,i};
            if(isStart(loc) ) {
                startSlots.push_back(loc);
            }
        }
    }
    
}

void SwarmWorld::evaluateSolo(shared_ptr<Organism> org, int analyse, int visualize, int debug) {
    int maxOrgs = startSlots.size() * nAgents;
    //visualize = 0;
    //mt19937 generator(Global::randomSeedPL->lookup());
    
    //vector<int> startSlotsRandom;
    //for (int i=0; i<startSlots.size(); i++) startSlotsRandom.push_back(i);
    //shuffle ( startSlotsRandom.begin(), startSlotsRandom.end(), generator);
    
    //uniform_int_distribution<> facing_dis(0,3);
    
    collisionCount = 0;
    
    vector<vector<vector<string>>> worldLog;
    vector<vector<int>> states;
    vector<int> states_count;
    vector<vector<int>> oldStates;
    
    //if(phero) this->pheroMap = SwarmWorld::zerosDouble(this->gridX, this->gridY);
    this->agentMap = SwarmWorld::zeros(this->gridX, this->gridY);
    
    // INIT LOG
    if(visualize) {
        for (int i = 0; i < maxOrgs; i++) {
            worldLog.push_back(vector<vector<string>>());
            worldLog[i].push_back(vector<string>()); // X
            worldLog[i].push_back(vector<string>()); // Y
            worldLog[i].push_back(vector<string>()); // F
            worldLog[i].push_back(vector<string>()); // S
            for (int j = 0; j < worldUpdates; j++) {
                worldLog[i][0].push_back("-1");
                worldLog[i][1].push_back("-1");
                worldLog[i][2].push_back("1");
                worldLog[i][3].push_back("0");
            }
            
        }
        
    }
    
    
    // PLACE AGENTS
    
    org->brain->resetBrain();
    for (int idx = 0; idx < maxOrgs; idx++) {
        
        location.push_back({-1,-1});
        oldLocation.push_back({-1,-1});
        score.push_back(0);
        
        facing.push_back(START_FACING[idx % 4]);
        
        
        waitForGoal.push_back(0);
        oldStates.push_back(vector<int>());
        
        move(idx,startSlots[int(idx*(startSlots.size()/float(maxOrgs)))], 1);
    }
    
    int nNodes = (int)dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes.size();
    vector<int> orgsRandom;
    for (int t = 0; t < worldUpdates; t++) {
        //if(phero) decay();
        //cout << "\n";
        
        
        //orgsRandom.clear();
        //for (int idxRand=0; idxRand<maxOrgs; idxRand++) orgsRandom.push_back(idxRand);
        //shuffle ( orgsRandom.begin(), orgsRandom.end(), generator);
        
        
        for (int idx = 0; idx < maxOrgs; idx++) {
            //int idx = orgsRandom[idxA];
            
            // SET SHARED BRAIN TO OLD STATE
            if(oldStates[idx].size()==nNodes) {
                for(int i = 0; i < nNodes ; i++) {
                    dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes[i] = oldStates[idx][i];
                }
            }
            
            // RESET OUTPUTS TO ZERO, TO AVOID CONNECTIONS FROM OUTPUT TO HIDDEN/INPUT
            if(resetOutputs) {
                org->brain->setOutput(0, 0);
                org->brain->setOutput(1, 0);
                dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes[requiredInputs()] = 0;
                dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes[requiredInputs() + 1] = 0;
            }
            
            int f = facing[idx];
            int stimulis = 0;
            vector<int> o_inputs;
            for(int i=0; i < senseSides.size(); i++) {
                pair<int,int> loc = getRelativePosition(location[idx], facing[idx], senseSides[i]);
                o_inputs.push_back(canMove(loc));
                
                if(senseAgents) o_inputs.push_back(hiddenAgents?0:isAgent(loc));
            }
            /*if(phero) {
             for(int i = 1; i <= 4; i++) {
             pair<int,int> loc = getRelativePosition(location[idx], facing[idx], i);
             if(isValid(loc))
             //o_inputs.push_back(Random::P(pheroMap[loc.first][loc.second]));
             o_inputs.push_back(pheroMap[loc.first][loc.second] > 0.5);
             }
             }*/
            
            
            for(int j = 0; j < o_inputs.size(); j++) {
                dynamic_pointer_cast<MarkovBrain>(org->brain)->setInput(j, o_inputs[j]);
                //stimulis += o_inputs[j];
                //cout << inputs[j] << " ";
            }
            
            //cout << stimulis << " stimulis for organism " << orgIndex << " set \n";
            
            // UPDATE BRAINS
            dynamic_pointer_cast<MarkovBrain>(org->brain)->update();
            vector<int> outputs;
            
            for(int i=0; i < requiredOutputs(); i++) {
                outputs.push_back(Bit(org->brain->readOutput(i)));
            }
            int new_dir = 0;
            
            if(outputs[0] == 1 &&  outputs[1] == 0) {
                f = (f - 2) % 8;
                if (f < 0) f+=8;
            } else if (outputs[0] == 0 &&  outputs[1] == 1) {
                f = (f + 2) % 8;
                if (f < 0) f+=8;
            } else if (outputs[0] == 1 &&  outputs[1] == 1) {
                new_dir = 1;
            }
            
            facing[idx] = f;
            
            
            if(new_dir != 0) {
                pair<int,int> new_pos = getRelativePosition(location[idx], facing[idx], new_dir);
                if(canMove(new_pos)) {
                    move(idx, new_pos, f);
                }
            }
            // SET SHARED BRAIN TO OLD STATE
            
            //for(int i = 0; i < oldStates[idx].size(); i++) {
            //    oldStates[idx][i].clear();
            //}
            
            oldStates[idx].clear();
            for(int idxState = 0; idxState < nNodes; idxState++) {
                oldStates[idx].push_back(dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes[idxState]);
            }
            for (int i = 0; i < o_inputs.size(); i++) {
                oldStates[idx][i] = o_inputs[i];
            }
        }
        
        if(visualize) {
            
            // TRACK POSITIONS
            for(int i = 0; i < maxOrgs; i++) {
                worldLog[i][0][t] = (to_string(location[i].first));
                worldLog[i][1][t] = (to_string(location[i].second));
                worldLog[i][2][t] = (to_string(facing[i]));
                worldLog[i][3][t] = (to_string(score[i]));
                
                
                //TRACK STATES
                vector<int> state = oldStates[i];
                for(int j = 0; j < state.size(); j++) state[j] = ((int)state[j] > 0);
                
                bool f = false;
                int f_idx = -1;
                for(int j = 0; j < states.size(); j++) {
                    f = true;
                    for(int k = 0; k < states[j].size(); k++) {
                        if(states[j][k] != state[k]) {
                            f = false;
                            break;
                        }
                    }
                    if(f) {
                        f_idx = j;
                        break;
                    }
                }
                if(f_idx != -1) {
                    states_count[f_idx]++;
                } else {
                    states.push_back(state);
                    states_count.push_back(1);
                }
                
            }
            
            
        }
    }
    
    // CALCULATE SCORE
    double globalscore = 0;
    for (int i = 0; i < maxOrgs; i++) {
        globalscore += score[i];
    }
    globalscore /= maxOrgs;
    org->dataMap.setOutputBehavior("score", DataMap::AVE | DataMap::LIST);
    org->dataMap.Append("score", globalscore);
    
    if (visualize) {
        // WRITE BEST BRAIN TPM/CM
        shared_ptr<MarkovBrain> mb = dynamic_pointer_cast<MarkovBrain>(org->brain->makeCopy());
        getTPM(mb);
        getCM(mb);
        
        // WRITE STATES
        //SORT
        
        for(int i = 0; i < states.size(); i++) {
            for(int j = 0; j < states.size(); j++) {
                if(states_count[j] > states_count[i]) {
                    int c = states_count[j];
                    states_count[j] = states_count[i];
                    states_count[i] = c;
                    vector<int> s = states[j];
                    states[j] = states[i];
                    states[i] = s;
                }
            }
        }
        
        stringstream ssfile;
        ssfile << FileManager::outputDirectory << "/states.csv";
        string states_file = ssfile.str();
        
        ofstream sfile;
        sfile.open (states_file);
        
        for(int i = 0; i < states.size(); i++) {
            for(int j = 0; j < states[i].size(); j++) {
                sfile << states[i][j];
                if(j<states[i].size()-1) sfile << ",";
            }
            sfile << "\n";
        }
        sfile.close();
        
        stringstream ss_statesfile;
        ss_statesfile << FileManager::outputDirectory << "/states_count.csv";
        states_file = ss_statesfile.str();
        
        sfile.open (states_file);
        
        for(int i = 0; i < states_count.size(); i++) {
            sfile << states_count[i] << "\n";
        }
        sfile.close();
        
        // WRITE POSITONS
        stringstream ss;
        ss << FileManager::outputDirectory << "/positions.csv";
        string pos_file = ss.str();
        
        ofstream map;
        map.open (pos_file);
        for (int i = 0; i < 4; i++) {
            for(int j = 0; j < maxOrgs; j++) {
                stringstream val;
                val << '"';
                for(int k = 0; k<worldUpdates; k++) {
                    if (k+1 >= worldUpdates) {
                        val << worldLog[j][i][k];
                    } else {
                        val << worldLog[j][i][k] << '|';
                        
                    }
                }
                val << '"';
                
                if (j+1 >= maxOrgs) {
                    map << val.str();
                } else {
                    map << val.str() << ',';
                    
                }
            }
            map << "\n";
        }
        map.close();
        
        //write score
        double scorewocol = globalscore;
        if(hasPenalty) {
            scorewocol = scorewocol + ((penalty * collisionCount) / maxOrgs);
        }
        stringstream scorefile_ss;
        scorefile_ss << FileManager::outputDirectory << "/score.csv";
        string scorefile = scorefile_ss.str();
        ofstream scorefile_of;
        scorefile_of.open (scorefile);
        scorefile_of << globalscore << ";" << collisionCount << ";" << scorewocol;
        
        scorefile_of.close();
        
        
    }
    
    generation++;
    
    // CLEAN UP
    for (int i = 0; i < gridX ; ++i){
        delete [] this->agentMap[i];
        //if(phero) delete [] this->pheroMap[i];
    }
    
    location.clear();
    oldLocation.clear();
    score.clear();
    waitForGoal.clear();
    facing.clear();
    for(int i = 0; i < oldStates.size(); i++) {
        oldStates[i].clear();
    }
    oldStates.clear();
    
}

int SwarmWorld::requiredInputs() {
    return (senseSides.size() * (senseAgents?2:1));//+ (phero?4:0); // moving + bridge + goal + comm
    //return groupSize * 12;
}
int SwarmWorld::requiredOutputs() {
    return (2) ;
}

int** SwarmWorld::zeros(int x, int y) {
    int** m = new int*[x];
    
    for (int i = 0; i < x; i++) {
        m[i] = new int[y];
        for(int j = 0; j < y; j++) {
            m[i][j] = 0;
        }
    }
    
    return m;
}

double** SwarmWorld::zerosDouble(int x, int y) {
    double** m = new double*[x];
    
    for (int i = 0; i < x; i++) {
        m[i] = new double[y];
        for(int j = 0; j < y; j++) {
            m[i][j] = 0;
        }
    }
    
    return m;
}


void SwarmWorld::decay() {
    
    for (int i = 0; i < gridX; i++) {
        for(int j = 0; j < gridY; j++) {
            pheroMap[i][j] *= 0.7;
        }
    }
}


bool SwarmWorld::isValid(pair<int,int> loc) {
    if (loc.first < 0) return false;
    if (loc.second < 0) return false;
    if (loc.first  >= gridX) return false;
    if (loc.second >= gridY) return false;
    return true;
    
}

bool SwarmWorld::isWater(pair<int,int> loc) {
    if(!isValid(loc)) return false;
    if(this->waterMap[loc.first][loc.second] == 2) {
        return true;
    } else {
        return false;
    }
}
bool SwarmWorld::isGoal(pair<int,int> loc) {
    return isValid(loc) && this->waterMap[loc.first][loc.second] == 4;
}
bool SwarmWorld::isFloor(pair<int,int> loc) {
    return isValid(loc) && this->waterMap[loc.first][loc.second] == 1;
}
bool SwarmWorld::isStart(pair<int,int> loc) {
    return isValid(loc) && this->waterMap[loc.first][loc.second] == 3;
}

pair<int,int> SwarmWorld::getRelativePosition(pair<int,int> loc, int facing, int direction) {
    int dir = ((facing + direction - 1) % 8) - 1;
    if (dir == -1) dir = 7;
    int x = loc.first + SwarmWorld::RELPOS[dir][0];
    int y = loc.second + SwarmWorld::RELPOS[dir][1];
    return {x, y};
}
/*
 pair<int,int> SwarmWorld::isGoalInSight(pair<int,int>loc, int facing) {
 pair<int,int> ret;
 ret.first = 0;
 ret.second = 0;
 switch(facing) {
 case 1:
 if (loc.first < avgGoal.first && loc.second <= avgGoal.second) ret.first = 1;
 if (loc.first < avgGoal.first && loc.second >= avgGoal.second) ret.second = 1;
 break;
 case 3:
 if (loc.first <= avgGoal.first && loc.second > avgGoal.second) ret.first = 1;
 if (loc.first >= avgGoal.first && loc.second > avgGoal.second) ret.second = 1;
 break;
 case 5:
 if (loc.first > avgGoal.first && loc.second <= avgGoal.second) ret.first = 1;
 if (loc.first < avgGoal.first && loc.second >= avgGoal.second) ret.second = 1;
 break;
 case 7:
 if (loc.first >= avgGoal.first && loc.second > avgGoal.second) ret.first = 1;
 if (loc.first <= avgGoal.first && loc.second > avgGoal.second) ret.second = 1;
 break;
 }
 return ret;
 }*/


int SwarmWorld::distance(pair<int, int> a, pair<int,int> b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}


int** SwarmWorld::loadLevel() {
    // simple path from left to right
    int** mat = zeros(gridX, gridY);
    
    string fileName = "./level.csv";
    std::ifstream file(fileName);
    
    for(int row = 0; row < gridY; ++row)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            break;
        
        std::stringstream iss(line);
        
        for (int col = 0; col < gridX; ++col)
        {
            std::string val;
            std::getline(iss, val, ' ');
            if ( !iss.good() )
                break;
            
            std::stringstream convertor(val);
            convertor >> mat[col][row];
        }
    }
    
    
    return mat;
}

void SwarmWorld::showMat(int** mat, int x, int y) {
    for (int i = 0; i < y; i++) {
        for(int j = 0; j < x; j++) {
            if (mat[j][i] > -1) {
                
                cout << ' ' << mat[j][i] << ' ';
            } else {
                cout << mat[j][i] << ' ';
                
            }
        }
        cout << "\n";
    }
    cout << "\n";
}

void SwarmWorld::writeMap() {
    
    ofstream map;
    std::remove("map.csv");
    map.open ("map.csv",ios::app);
    for (int i = 0; i < gridY; i++) {
        for(int j = 0; j < gridX; j++) {
            if (j+1 >= gridX) {
                map << waterMap[j][i];
            } else {
                map << waterMap[j][i] << ',';
                
            }
        }
        map << "\n";
    }
    map.close();
}

double ** SwarmWorld::getTPM(shared_ptr<MarkovBrain> brain) {
    
    // EXPECT THAT HIDDEN NODES ARE IN THE END OF THE NODE LIST (VERIFIED)
    int n = brain->nrNodes;
    int n_states = pow(2,n);
    double** mat = zerosDouble(n, n_states);
    
    
    for (int i = 0; i < n_states; i++) {
        brain->resetBrain();
        
        int* array = new int[32];
        for (int j = 0; j < 32; ++j) {  // assuming a 32 bit int
            array[j] = i & (1 << j) ? 1 : 0;
            //cout << (i & (1 << j) ? 1 : 0);
        }
        //cout << "\n";
        
        for(int j = 0; j < n; j++) {
            if(j < brain->inputValues.size()) {
                brain->inputValues[j] = array[j];
            } else if (resetOutputs && j>=brain->inputValues.size() && j < brain->inputValues.size() + 2) {
                // MAKE SURE THAT OUTPUTS WILL NOT CAUSE ANYTHING (PYPHI-STUFF)
                brain->nodes[j] = 0;
            } else {
                // HIDDEN NODES
                brain->nodes[j] = array[j];
            }
        }
        brain->update();
        for(int j = 0; j < n; j++) {
            int val = brain->nodes[j];
            if(j < brain->inputValues.size()) {
                val = array[j];
            }
            mat[j][i] = (val>0?1:0);
        }
    }
    
    ofstream map;
    stringstream ss;
    ss << FileManager::outputDirectory << "/tpm.csv";
    map.open (ss.str());
    for (int i = 0; i < n_states; i++) {
        for(int j = 0; j < n; j++) {
            if (j+1 >= n) {
                map << mat[j][i];
            } else {
                map << mat[j][i] << ' ';
                
            }
        }
        map << "\n";
    }
    map.close();
    
    return mat;
}


vector<vector<int>> SwarmWorld::getCM(shared_ptr<MarkovBrain> brain) {
    
    
    int n = brain->nrNodes;
    vector<vector<int>> mat = brain->getConnectivityMatrix();
    
    ofstream map;
    stringstream ss;
    ss << FileManager::outputDirectory << "/cm.csv";
    map.open (ss.str());
    for (int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            //cout << mat[i][j] << " ";
            int val = mat[i][j] > 0;
            // DO NOT ALLOW CONNECTIONS TO INPUTS
            if(j < requiredInputs()) val = 0;
            //if(i >= requiredInputs() && i < requiredInputs() + requiredOutputs()) val = 0;
            
            if (j+1 >= n) {
                map << val;
            } else {
                map << val << ' ';
                
            }
        }
        //cout << "\n";
        map << "\n";
    }
    map.close();
    
    return mat;
}


void SwarmWorld::move(int idx, pair<int,int> newloc, int dir) {
    
    if(isGoal(newloc) && waitForGoal[idx]<=0) {
        score[idx] +=1;
        waitForGoal[idx] = waitForGoalI;
    }
    waitForGoal[idx] --;
    if(isAgent(newloc)) {
        collisionCount++;
        if(hasPenalty) score[idx] -=penalty;
        if(blockWay) return;
    }
    oldLocation[idx] = location[idx];
    location[idx] = newloc;
    
    agentMap[newloc.first][newloc.second] += 1;
    //if(phero) pheroMap[newloc.first][newloc.second] = 1;
    if(oldLocation[idx].first > 0 && oldLocation[idx].second > 0) {
        agentMap[oldLocation[idx].first][oldLocation[idx].second] -= 1;
    }
    
}

bool SwarmWorld::isAgent(pair<int,int> loc) {
    if(!isValid(loc)) return false;
    
    if(agentMap[loc.first][loc.second] > 0) {
        return true;
    } else {
        return false;
    };
}




bool SwarmWorld::canMove(pair<int,int> locB) {
    bool move = true;
    if(!isValid(locB)) {
        move = false;
    } else if(isWall(locB)) {
        move = false;
    }
    return move;
}


bool SwarmWorld::isWall(pair<int, int> loc) {
    return isValid(loc) && waterMap[loc.first][loc.second] == 0;
}


