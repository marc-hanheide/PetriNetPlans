// Standalone version of PNP generation from policy

#include<iostream>
#include<fstream>
#include <stack>

#include "policy.h"
#include "pnpgenerator.h"

using namespace std;



void create_PNP_from_policy(const string& polfile_n, const string& erfile) {
    Policy p;
    if (polfile_n!="") {
      p.goal_name = polfile_n;
      p.initial_state = "initial_state";
      p.final_state = "CurrentNode_WayPoint4";

      string line;
      ifstream polfile;
      polfile.open(polfile_n, ios::in);

      if (polfile.is_open()) {

        vector<StateOutcome> conditions;
        vector<string> actions;
        vector<string> states;
        while ( getline (polfile, line) ) {
          string state = line.substr(0, line.find("\t"));
          string action = line.substr(line.find("\t") + 1, line.length() - line.find("\t") - 1);
          cout << state << endl;
          states.push_back(state);
          actions.push_back(action);
          conditions.push_back(StateOutcome(state, state));
          //cout << line << '\n';
        }
        p.addStatePolicy("initial_state", "say_hello", conditions);
        for (int i=0; i<actions.size(); i++)
          p.addStatePolicy(states[i], actions[i], conditions);
        polfile.close();
      } else
        cout << "Unable to open file";

    } else {

      // ...
      p.goal_name = "SimplePolicy";
      p.initial_state = "S0";
      p.final_state = "S3";
      // ...

      vector<StateOutcome> v0; v0.push_back(StateOutcome("personhere","S1")); v0.push_back(StateOutcome("not personhere","S2"));
      p.addStatePolicy("S0","goto_printer",v0);

      vector<StateOutcome> v1; v1.push_back(StateOutcome("","S2"));
      p.addStatePolicy("S1","say_hello",v1);

      vector<StateOutcome> v2; v2.push_back(StateOutcome("","S3"));
      p.addStatePolicy("S2","goto_home",v2);

    }
    p.print();



    // Generates the PNP

    cout << "Policy name: " << p.goal_name << endl;
    PNPGenerator pnpgen(p.goal_name);

    // generate the PNP from the policy
    bool r=pnpgen.genFromPolicy(p);

    if (r) {

        if (erfile!="") {
            // apply the execution rules
            pnpgen.readERFile(erfile);
            pnpgen.applyExecutionRules();
        }

        string pnpoutfilename = p.goal_name+".pnml";
        pnpgen.save(pnpoutfilename.c_str());
        cout << "Saved PNP file " << pnpoutfilename << endl;
    }
    else {
        cout << "PNP not generated!!!" << endl;
    }
}


int main(int argc, char** argv) {

    string polfile_n="";
    string erfile="";
    if (argc>1)
      polfile_n = string(argv[1]);
    if (argc>2)
      erfile = string(argv[2]);


    create_PNP_from_policy(polfile_n, erfile);

}
