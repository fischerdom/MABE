
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <vector>

#include "Swarm2World.h"
#include "../../Organism/Organism.h"

shared_ptr<ParameterLink<int>> Swarm2World::gridXSizePL = Parameters::register_parameter("WORLD_SWARM2-gridX", 32, "size of grid X");
shared_ptr<ParameterLink<int>> Swarm2World::gridYSizePL = Parameters::register_parameter("WORLD_SWARM2-gridY", 32, "size of grid Y");
shared_ptr<ParameterLink<int>> Swarm2World::worldUpdatesPL = Parameters::register_parameter("WORLD_SWARM2-worldUpdates", 500, "amount of time a brain is tested");
shared_ptr<ParameterLink<double>> Swarm2World::nAgentsPL = Parameters::register_parameter("WORLD_SWARM2-nAgents", 1.0, "how many agents in a game in rate");
shared_ptr<ParameterLink<int>> Swarm2World::senseAgentsPL = Parameters::register_parameter("WORLD_SWARM2-senseAgents", 0, "1 if ants can sense");
shared_ptr<ParameterLink<string>> Swarm2World::senseSidesPL = Parameters::register_parameter("WORLD_SWARM2-senseSides", (string)"[1]", "1 if ants can sense");
shared_ptr<ParameterLink<int>> Swarm2World::resetOutputsPL = Parameters::register_parameter("WORLD_SWARM2-resetOutputs", 1, "1 if outputs should be reseted after one time step");
shared_ptr<ParameterLink<int>> Swarm2World::hasPenaltyPL = Parameters::register_parameter("WORLD_SWARM2-hasPenalty", 1, "1 if penalty when agents get hit");
shared_ptr<ParameterLink<double>> Swarm2World::penaltyPL = Parameters::register_parameter("WORLD_SWARM2-penalty", 0.075, "amount of penalty for hit");
shared_ptr<ParameterLink<int>> Swarm2World::waitForGoalPL = Parameters::register_parameter("WORLD_SWARM2-waitForGoal", 100, "timestep till the next goal is possible");

Swarm2World::Swarm2World(shared_ptr<ParametersTable> _PT) : AbstractWorld(_PT) {
    cout << "Using Swarm2World \n";
    
    worldUpdates = (PT == nullptr) ? worldUpdatesPL->lookup() : PT->lookupInt("WORLD_SWARM2-worldUpdates");
    gridX = (PT == nullptr) ? gridXSizePL->lookup() : PT->lookupInt("WORLD_SWARM2-gridX");
    gridY = (PT == nullptr) ? gridYSizePL->lookup() : PT->lookupInt("WORLD_SWARM2-gridY");
    senseAgents = ((PT == nullptr) ? senseAgentsPL->lookup() : PT->lookupInt("WORLD_SWARM2-senseAgents")) == 1;
    resetOutputs = ((PT == nullptr) ? resetOutputsPL->lookup() : PT->lookupInt("WORLD_SWARM2-resetOutputs")) == 1;
    hasPenalty = ((PT == nullptr) ? hasPenaltyPL->lookup() : PT->lookupInt("WORLD_SWARM2-hasPenalty")) == 1;
    nAgents = ((PT == nullptr) ? nAgentsPL->lookup() : PT->lookupDouble("WORLD_SWARM2-nAgents"));
    convertCSVListToVector(((PT == nullptr) ? senseSidesPL->lookup() : PT->lookupString("WORLD_SWARM2-senseSides")), senseSides);
    penalty = (PT == nullptr) ? penaltyPL->lookup() : PT->lookupDouble("WORLD_SWARM2-penalty");
    waitForGoalI = (PT == nullptr) ? waitForGoalPL->lookup() : PT->lookupInt("WORLD_SWARM2-waitForGoal");
    

    generation = 0;

    cout << worldUpdates << " Updates\n";
    
    
    // columns to be added to ave file
    aveFileColumns.clear();
    aveFileColumns.push_back("score");
    std::remove("positions.csv");
    
    this->buildGrid();
    
}

void Swarm2World::buildGrid(){
    
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

void Swarm2World::evaluateSolo(shared_ptr<Organism> org, int analyse, int visualize, int debug) {
    srand(1234);
    //int maxOrgs = startSlots.size() * nAgents;
    int maxOrgs = 25;
    
    vector<vector<vector<string>>> worldLog;
    vector<vector<int>> states;
    vector<int> states_count;
    vector<vector<int>> oldStates;
    
    this->agentMap = Swarm2World::zeros(this->gridX, this->gridY);
    this->pheroMap = Swarm2World::zerosDouble(this->gridX, this->gridY);
    
    int mtp = 0, mtr = 0, mta = 0, mtg = 0, mtd = 0;
    
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
        waitForGoal.push_back(0);
        oldStates.push_back(vector<int>());
        
        move(idx,startSlots[idx % startSlots.size()]); // only one start slot
    }
    
    int nNodes = dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes.size();
    for (int t = 0; t < worldUpdates; t++) {
        decay();
        //cout << "\n";
        for (int idx = 0; idx < maxOrgs; idx++) {
            if(idx != 0 && score[idx - 1] < 1) break;
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
            
            
            int stimulis = 0;
            vector<int> o_inputs;
            /*
             Input Coding
             0: Phero
             1: Neighbor
             2: Ditch
             3: Goal (depends if food picked or not, if food picked, start slot is goal)
             */
            pair<int,int> cl = location[idx];
            o_inputs.push_back(hasPhero(cl));
            o_inputs.push_back(hasAgent(cl));
            //o_inputs.push_back(hasDitch(cl));
            o_inputs.push_back(waitForGoal[idx] <= 0?hasGoal(cl):hasStart(cl));
            
            
            
            for(int j = 0; j < o_inputs.size(); j++) {
                dynamic_pointer_cast<MarkovBrain>(org->brain)->setInput(j, o_inputs[j]);
                stimulis += o_inputs[j];
                //cout << inputs[j] << " ";
            }
            
            //cout << stimulis << " stimulis for organism " << orgIndex << " set \n";
            
            // UPDATE BRAINS
            dynamic_pointer_cast<MarkovBrain>(org->brain)->update();
            vector<int> outputs;
            
            for(int i=0; i < requiredOutputs(); i++) {
                outputs.push_back(Bit(org->brain->readOutput(i)));
            }
            
            if(!isBridge(idx)) {
                if(outputs[0] == 0 &&
                   outputs[1] == 0 ) {
                   //outputs[2] == 0 ) {
                    moveToRandom(idx);
                    mtr +=1;
                } else if(outputs[0] == 1 &&   // Move to Phero
                          outputs[1] == 0 ) {
                    //outputs[2] == 0 ) {
                   moveToPhero(idx);
                    mtp +=1;
                    //cout << "P";
                } else if (outputs[0] == 0 &&   // Move to Agent
                           outputs[1] == 1 ) {
                    //outputs[2] == 0 ) {
                    moveToAgent(idx);
                    mta +=1;
                    //cout << "A";
                }/*else if (outputs[0] == 1 &&   // Move to Ditch
                           outputs[1] == 1 &&
                           outputs[2] == 0) {
                    moveToDitch(idx);
                    //cout << "D";
                } */else if (outputs[0] == 1 &&   // Move to Goal
                             outputs[1] == 1 ) {
                    //outputs[2] == 0 ) {
                    moveToGoal(idx);
                    mtg +=1;
                    //cout << "G";
                }/*else if (outputs[0] == 1 &&   // Move to Goal
                           outputs[1] == 1 &&
                           outputs[2] == 1) {
                    moveToRandom(idx);
                    //cout << "R";
                } else {
                    // nothing
                }*/
            }
            
            // update phero
            vector<pair<int, int>> plocs = getInnerPosition(location[idx]);
            
            for(int p_i=0;p_i < plocs.size(); p_i++) {
                if(pheroMap[plocs[p_i].first][plocs[p_i].second] > 1) {
                    pheroMap[plocs[p_i].first][plocs[p_i].second] *= 1/pheroMap[plocs[p_i].first][plocs[p_i].second];
                } else {
                    pheroMap[plocs[p_i].first][plocs[p_i].second] += 0.3;
                }
            }
            if(pheroMap[location[idx].first][location[idx].second] > 1) pheroMap[location[idx].first][location[idx].second] *= 1/pheroMap[location[idx].first][location[idx].second];
            else pheroMap[location[idx].first][location[idx].second] += 0.7;
        
            
            // CLEAN AUP MARKOV BRAIN
            oldStates[idx].clear();
            for(int i = 0; i < nNodes; i++) {
                oldStates[idx].push_back(dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes[i]);
            }
        }
        
        
        
        
        // TRACK POSITIONS
        if(visualize) {
            for(int i = 0; i < maxOrgs; i++) {
                worldLog[i][0][t] = (to_string(location[i].first));
                worldLog[i][1][t] = (to_string(location[i].second));
                //worldLog[i][2][t] = (to_string(facing[i]));
                worldLog[i][2][t] = (to_string(0));
                worldLog[i][3][t] = (to_string(score[i]));
                
                
               
                vector<int> state;
                for(int i = 0; i < nNodes; i++) {
                    state.push_back(dynamic_pointer_cast<MarkovBrain>(org->brain)->nodes[i]>0?1:0);
                }
                bool f = false;
                int f_idx = -1;
                for(int i = 0; i < states.size(); i++) {
                    f = true;
                    for(int j = 0; j < states[i].size(); j++) {
                        if(states[i][j] != state[j]) {
                            f = false;
                            break;
                        }
                    }
                    if(f) {
                        f_idx = i;
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
    double maxscore = 0;
    for (int i = 0; i < maxOrgs; i++) {
        globalscore += score[i];
        if (score[i] > maxscore) {
            maxscore = score[i];
        }
    }
    double gp = 0.0;
    for (int i = 0; i < gridX; i++) {
        for (int j = 0; j < gridY; j++) {
            gp+= pheroMap[i][j];
        }
    }
    //cout << gp << " " << globalscore <<"\n";
    globalscore /= maxOrgs;
    //cout << globalscore << " -- " << mtp << " " << mtr << " " << mta << " " << mtg << " " << mtd << "\n";
    
    org->dataMap.setOutputBehavior("score", DataMap::AVE | DataMap::LIST);
    //org->dataMap.Append("score", globalscore);
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
        
    }
    
    generation++;
    
    // CLEAN UP
    for (int i = 0; i < gridX ; ++i){
        delete [] this->agentMap[i];
        delete [] this->pheroMap[i];
    }
    
    location.clear();
    oldLocation.clear();
    score.clear();
    waitForGoal.clear();
    stack.clear();
    for(int i = 0; i < oldStates.size(); i++) {
        oldStates[i].clear();
    }
    oldStates.clear();
    
}

int Swarm2World::requiredInputs() {
    return 3;
}
int Swarm2World::requiredOutputs() {
    return 2;
}

int** Swarm2World::zeros(int x, int y) {
    int** m = new int*[x];
    
    for (int i = 0; i < x; i++) {
        m[i] = new int[y];
        for(int j = 0; j < y; j++) {
            m[i][j] = 0;
        }
    }
    
    return m;
}

bool Swarm2World::isValid(pair<int,int> loc) {
    if (loc.first < 0) return false;
    if (loc.second < 0) return false;
    if (loc.first  >= gridX) return false;
    if (loc.second >= gridY) return false;
    return true;
    
}

bool Swarm2World::isWater(pair<int,int> loc) {
    if(!isValid(loc)) return false;
    if(this->waterMap[loc.first][loc.second] == 2) {
        return true;
    } else {
        return false;
    }
}
bool Swarm2World::hasGoal(pair<int,int> loc) {
    bool isGoal = false;
    if (isValid(loc)) {
        vector<pair<int, int>> locsInner = getInnerPosition(loc);
        
        for(int i = 0; i < locsInner.size(); i++) {
            pair<int, int> searchloc = locsInner[i];
            if(this->waterMap[searchloc.first][searchloc.second] == 4) {
                isGoal = true;
                break;
            }
        }
    }
    
    return isGoal;
}
bool Swarm2World::hasStart(pair<int,int> loc) {
    bool isStart = false;
    if (isValid(loc)) {
        vector<pair<int, int>> locsInner = getInnerPosition(loc);
        
        for(int i = 0; i < locsInner.size(); i++) {
            pair<int, int> searchloc = locsInner[i];
            if(this->waterMap[searchloc.first][searchloc.second] == 3) {
                isStart = true;
                break;
            }
        }
    }
    
    return isStart;
}

bool Swarm2World::isBridge(int idx) {
    return !stack.empty() && find(begin(stack),end(stack),idx) != end(stack);
}

bool Swarm2World::hasDitch(pair<int,int> loc) {
    bool isDitch = false;
    if (isValid(loc)) {
        vector<pair<int, int>> locsInner = getInnerPosition(loc);
        
        for(int i = 0; i < locsInner.size(); i++) {
            pair<int, int> searchloc = locsInner[i];
            if(this->waterMap[searchloc.first][searchloc.second] == 2) {
                isDitch = true;
                break;
            }
        }
    }
    
    return isDitch;
}

bool Swarm2World::hasAgent(pair<int,int> loc) {
    bool isAgent = false;
    if(!isValid(loc)) return false;
    
    vector<pair<int, int>> locsOuter = getInnerPosition(loc);
    
    for(int i = 0; i < locsOuter.size(); i++) {
        pair<int, int> searchloc = locsOuter[i];
        if(agentMap[searchloc.first][searchloc.second] > 0) {
            isAgent = true;
            break;
        }
    }
    
    return isAgent;
}

bool Swarm2World::hasPhero(pair<int,int> loc) {
    bool isPhero = false;
    if(!isValid(loc)) return false;
    
    vector<pair<int, int>> locsInner = getInnerPosition(loc);
    vector<pair<int, int>> locsOuter = getInnerPosition(loc);
    
    for(int i = 0; i < locsInner.size(); i++) {
        pair<int, int> searchloc = locsInner[i];
        if(pheroMap[searchloc.first][searchloc.second] > 0) {
            isPhero = true;
            break;
        }
    }
    for(int i = 0; i < locsOuter.size(); i++) {
        pair<int, int> searchloc = locsOuter[i];
        if(pheroMap[searchloc.first][searchloc.second] > 0) {
            isPhero = true;
            break;
        }
    }
    
    return isPhero;
}
bool Swarm2World::isStart(pair<int,int> loc) {
    return isValid(loc) && this->waterMap[loc.first][loc.second] == 3;
}
bool Swarm2World::isGoal(pair<int,int> loc) {
    return isValid(loc) && this->waterMap[loc.first][loc.second] == 4;
}

pair<int,int> Swarm2World::getRelativePosition(pair<int,int> loc, int facing, int direction) {
    int dir = ((facing + direction - 1) % 8) - 1;
    if (dir == -1) dir = 7;
    int x = loc.first + Swarm2World::RELPOS[dir][0];
    int y = loc.second + Swarm2World::RELPOS[dir][1];
    return {x, y};
}

vector<pair<int,int>> Swarm2World::getOuterPosition(pair<int,int> loc) {
    vector<pair<int,int>> locs;
    locs.push_back({loc.first + 2,loc.second});
    locs.push_back({loc.first + 2,loc.second + 1});
    locs.push_back({loc.first + 2,loc.second + 2});
    locs.push_back({loc.first + 2,loc.second + 2});
    locs.push_back({loc.first + 1,loc.second + 2});
    locs.push_back({loc.first,loc.second + 2});
    locs.push_back({loc.first - 1,loc.second + 2});
    locs.push_back({loc.first - 2,loc.second + 2});
    locs.push_back({loc.first - 2,loc.second + 1});
    locs.push_back({loc.first - 2,loc.second});
    locs.push_back({loc.first - 2,loc.second - 1});
    locs.push_back({loc.first - 2,loc.second - 2});
    locs.push_back({loc.first - 1,loc.second - 2});
    locs.push_back({loc.first,loc.second - 2});
    locs.push_back({loc.first + 1,loc.second - 2});
    locs.push_back({loc.first + 2,loc.second - 1});
    vector<pair<int,int>> locsValid;
    for(int i = 0; i < 16; i++) {
        if(isValid(locs[i])) {
            locsValid.push_back(locs[i]);
        }
    }
    return locsValid;
}
vector<pair<int,int>> Swarm2World::getInnerPosition(pair<int,int> loc) {
    vector<pair<int,int>> locs;
    locs.push_back({loc.first + 1, loc.second});
    locs.push_back({loc.first + 1, loc.second + 1});
    locs.push_back({loc.first, loc.second + 1});
    locs.push_back({loc.first - 1, loc.second + 1});
    locs.push_back({loc.first - 1, loc.second});
    locs.push_back({loc.first - 1, loc.second - 1});
    locs.push_back({loc.first, loc.second - 1});
    locs.push_back({loc.first + 1, loc.second - 1});
    vector<pair<int, int>> locsValid;
    for(int i = 0; i < 8; i++) {
        if(isValid(locs[i])) {
            locsValid.push_back(locs[i]);
        }
    }
    return locsValid;
}




int** Swarm2World::loadLevel() {
    // simple path from left to right
    int** mat = zeros(gridX, gridY);
    
    string fileName = "./level3.csv";
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

void Swarm2World::showMat(int** mat, int x, int y) {
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

void Swarm2World::writeMap() {
    
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

int ** Swarm2World::getTPM(shared_ptr<MarkovBrain> brain) {
    
    // EXPECT THAT HIDDEN NODES ARE IN THE END OF THE NODE LIST (VERIFIED)
    int n = brain->nrNodes;
    int n_states = pow(2,n);
    int** mat = zeros(n, n_states);
    
    
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
            //} else if (j>=brain->inputValues.size() && j < brain->inputValues.size() + 2) {
                // MAKE SURE THAT OUTPUTS WILL NOT CAUSE ANYTHING (PYPHI-STUFF)
            //    brain->nodes[j] = 0;
            } else {
                // HIDDEN NODES
                brain->nodes[j] = array[j];
            }
        }
        brain->update();
        for(int j = 0; j < n; j++) {
            int val = brain->nodes[j];
            
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


vector<vector<int>> Swarm2World::getCM(shared_ptr<MarkovBrain> brain) {
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
            // DO NOT ALLOW CONNECTIONS TO INPUTS OR FROM OUTPUTS TO SOMEWHERE
            if(j < requiredInputs()) val = 0;
            if(i >= requiredInputs() && i < requiredInputs() + requiredOutputs()) val = 0;
            
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

void Swarm2World::moveToGoal(int idx) {
    pair<int, int> loc = location[idx];
    vector<pair<int, int>> locs = getInnerPosition(loc);
    
    for(int i = 0; i < locs.size(); i++) {
        pair<int, int> searchloc = locs[i];
        if(!isWater(searchloc)) {
            if ((waitForGoal[idx] <= 0 && this->waterMap[searchloc.first][searchloc.second] == 4)
                || (waitForGoal[idx] > 0 && this->waterMap[searchloc.first][searchloc.second] == 3)) {
            move(idx,searchloc);
            return;
            }
        }
    }
}
void Swarm2World::moveToRandom(int idx) {
    pair<int, int> loc = location[idx];
    pair<int,int> newloc;
    vector<pair<int,int>> locs = getInnerPosition(loc);
    
    vector<int> myvector;
    for (int i=0; i<locs.size(); i++) myvector.push_back(i);
    random_shuffle ( myvector.begin(), myvector.end() );
    
    for (int i=0; i<locs.size(); ++i) {
        newloc = locs[myvector[i]];
        //newloc = locs[i];
        if(!isWater(newloc) && !isAgent(newloc)) {
            move(idx,newloc);
            return;
        }
    }
}

void Swarm2World::moveToDitch(int idx) {
    pair<int, int> loc = location[idx];
    vector<pair<int,int>> locs = getInnerPosition(loc);
    for(int i = 0; i < locs.size(); i++) {
        pair<int, int> searchloc = locs[i];
        if(isValid(searchloc) && this->waterMap[searchloc.first][searchloc.second] == 2) {
            bool canMoveIntoWater = false;
            vector<pair<int,int>> locs2 = getInnerPosition(searchloc);
            for(int j = 0; j < 8; j++) {
                pair<int, int> searchloc2 = locs2[j];
                if(searchloc2 != loc) {
                    if(isAgent(searchloc2) || isFloor(searchloc2)) {
                        canMoveIntoWater = true; break;
                    }
    
                }
            }
            if(canMoveIntoWater) {
                move(idx,searchloc);
                return;
            }
        }
    }
    
    
}

void Swarm2World::moveToAgent(int idx) {
    pair<int, int> loc = location[idx];
    vector<pair<int,int>> locs;
    if (isValid(loc)) {
        
        vector<pair<int,int>> ilocs = getInnerPosition(loc);
        for(int i = 0; i < ilocs.size(); i++) {
            pair<int, int> searchloc = ilocs[i];
            if(this->agentMap[searchloc.first][searchloc.second] == 0 && !isWater(searchloc)) {
                vector<pair<int,int>> ilocs2 = getInnerPosition(searchloc);
                for(int j = 0; j < ilocs2.size(); j++) {
                    pair<int, int> searchloc2 = ilocs2[j];
                    if(distance(searchloc2, loc) >=2 && this->agentMap[searchloc2.first][searchloc2.second] > 0) {
                        locs.push_back(searchloc);
                    }
                }
            } else if (this->agentMap[searchloc.first][searchloc.second] == 1 && isWater(searchloc)) {
                locs.push_back(searchloc);
            }
        }
        if(locs.size() > 0) {
            move(idx,locs[rand() % locs.size()]);
            //move(idx,locs[0]);
            return;
        }
    }
    
}

void Swarm2World::moveToPhero(int idx) {
    pair<int, int> loc = location[idx];
    double maxp = 0;
    pair<int,int> newpos;
    
    vector<pair<int, int>> locsInner = getInnerPosition(loc);
    
    for(int i = 0; i < locsInner.size(); i++) {
        pair<int, int> searchloc = locsInner[i];
        if(!isAgent(searchloc) && !isWater(searchloc)) {
            if(pheroMap[searchloc.first][searchloc.second] > maxp) {
                maxp = this->pheroMap[searchloc.first][searchloc.second];
                newpos = searchloc;
            }
            
            vector<pair<int, int>> locsInner2 = getInnerPosition(searchloc);
            for(int i = 0; i < locsInner2.size(); i++) {
                pair<int, int> searchloc2 = locsInner2[i];
                if(distance(loc, searchloc2) >= 2 && !isWater(searchloc2) && pheroMap[searchloc2.first][searchloc2.second] > maxp) {
                    maxp = this->pheroMap[searchloc2.first][searchloc2.second];
                    newpos = searchloc;
                }
            }
        }
    }
    
    if(maxp > 0) {
        move(idx,newpos);
    }

    
}

void Swarm2World::move(int idx, pair<int,int> newloc) {
    if (location[idx] != newloc) {
        //STACKING?
        if(isWater(location[idx]) && agentMap[location[idx].first][location[idx].second] == 2) {
            for(int i = 0; i < location.size(); i++) {
                if(i != idx && location[i].first == location[idx].first && location[i].second == location[idx].second) {
                    stack.erase(find(begin(stack),end(stack), i));
                    break;
                }
            }
        }
        if(isWater(newloc) && agentMap[newloc.first][newloc.second] == 1) {
            for(int i = 0; i < location.size(); i++) {
                if(location[i].first == newloc.first && location[i].second == newloc.second) {
                    stack.push_back(i);
                    break;
                }
            }
        }
        
        
        if(isGoal(newloc) && waitForGoal[idx]<=0) {
            score[idx] +=1;
            waitForGoal[idx] = 1;
        } else if (isStart(newloc) && waitForGoal[idx]>0) {
            score[idx] +=2;
            waitForGoal[idx] = 0;
        }
        
        oldLocation[idx] = location[idx];
        location[idx] = newloc;
        
        agentMap[newloc.first][newloc.second] += 1;
        
        
        if(oldLocation[idx].first > 0 && oldLocation[idx].second > 0) {
            agentMap[oldLocation[idx].first][oldLocation[idx].second] -= 1;
        }
    }
    
}

bool Swarm2World::isAgent(pair<int,int> loc) {
    if(!isValid(loc)) return false;
    
    if(agentMap[loc.first][loc.second] > 0) {
        return true;
    } else {
        return false;
    };
}


double** Swarm2World::zerosDouble(int x, int y) {
    double** m = new double*[x];
    
    for (int i = 0; i < x; i++) {
        m[i] = new double[y];
        for(int j = 0; j < y; j++) {
            m[i][j] = 0;
        }
    }
    
    return m;
}


bool Swarm2World::isFloor(pair<int,int> loc) {
    return isValid(loc) && this->waterMap[loc.first][loc.second] == 1;
}


void Swarm2World::decay() {
    
    for (int i = 0; i < gridX; i++) {
        for(int j = 0; j < gridY; j++) {
            pheroMap[i][j] *= 0.7;
            if (pheroMap[i][j] < 0.1) pheroMap[i][j]  = 0;
        }
    }
}


int Swarm2World::distance(pair<int, int> a, pair<int,int> b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}
