#include "condplan_translator.h"
#include "../pnpgenerator.h"
#include <map>

class DigraphTransl : public CondPlan_Translator{
private:
  string plan_name; //plan_#N of the digraph
  map<string,string> state_action;

public:
  DigraphTransl(string &path_to_file);
  DigraphTransl(string &path_to_file, string& pnml_out);
  void create_PNP(string& goal_name);
  void create_PNP(string& goal_name, vector<ConditionalPlan>& p);
  void read_file();
  void write_pnml(vector<ConditionalPlan>& v);
  vector<ConditionalPlan> get_plan(){ return this->p; }
};