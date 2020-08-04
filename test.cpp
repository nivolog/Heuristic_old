
#include <iostream>
#include <cmath>
#include <fstream>
#include <stack>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <typeinfo>
#include <ctime>
#include <list>
#include <chrono>

using namespace std;

int razmer1, razmer2;
double e = 1.0;
int chosen_one = 3;
// EUCLID == 1
// DIAG   == 2
// CHEB   == 3


class NODE {                                                                    //Main class
private:
	/* data */

public:
 int x_coordinate;                                                              //X position of NODE
 int y_coordinate;                                                              //Y position of NODE
 bool is_blocked;                                                               //1 if NODE is blocked - probably useless
 double g;                                                                      //distance to the start NODE
 double h;                                                                      //heuristic distance to the goal NODE
 double f;                                                                      //summ of g and h
 int id;
 NODE();                                                                        //constructors
 NODE(int x, int y);
 NODE *PRED;                                                                    //link to the predecessor




 void id_c(void){
   id = y_coordinate*razmer1 + x_coordinate;
 }
//compute cost
 double compute_cost(NODE &predecessor){                                                   //distance between two nodes
	int dx = x_coordinate - predecessor.x_coordinate;
	int dy = y_coordinate - predecessor.y_coordinate;
	int dd = fabs(dx*dy);
	if (dd > 0) return sqrt(2);                                                   //sqrt(2) in case of diagonal transition
	else return 1;                                                                //1 in case of horizontal/vertical transition
 }

 void g_calc(NODE &predecessor){
	g = predecessor.g + compute_cost(predecessor);
 }

 void h_calc(NODE &goal){
   switch (chosen_one){
      case 1:
        h = e * hypot(abs(x_coordinate - goal.x_coordinate), abs(y_coordinate - goal.y_coordinate));
        break;
      case 2:
        h = abs(x_coordinate - goal.x_coordinate) + abs(y_coordinate - goal.y_coordinate) + (sqrt(2) - 2)*fmin(abs(x_coordinate - goal.x_coordinate),abs(y_coordinate - goal.y_coordinate));
        h *= e;
        break;
      case 3:
        h = e * fmax(abs(x_coordinate - goal.x_coordinate), abs(y_coordinate - goal.y_coordinate));
        break;
      default:
        h = 0;
        break;
   }
 }


 void f_calc(){
	 f = g+h;
 }

 void check_g (double new_g){
	 g = fmin(g, new_g);
 }

};

NODE::NODE(void){                                                               //NODE constructor
  is_blocked = 0;
  g = 0;
  //PRED = nullptr;
}

NODE::NODE(int x, int y){                                                       //NODE constructor
  is_blocked = 0;
  x_coordinate = x;
  y_coordinate = y;
  id_c();
  //PRED = nullptr;
}

NODE n_start, n_goal;

struct comparator{
  using is_transparent = int;

  bool operator()(const NODE& m, const NODE& n) const{
     if (m.f < n.f){
       return true;
     }else if(m.f > n.f){
       return false;
     }else if(m.g < n.g){
       return true;
     }else if(m.g > n.g){
       return false;
     }else if(m.x_coordinate < n.x_coordinate){
       return true;
     }else if(m.x_coordinate > n.x_coordinate){
       return false;
     }else if(m.y_coordinate < n.y_coordinate){
       return true;
     }else return false;
 }
};


bool equivalent(NODE &one, NODE &two) {                                         //Test for two nodes being same (used for goal and start)
  if ((one.x_coordinate == two.x_coordinate)&&(one.y_coordinate == two.y_coordinate)) return true;
  else return false;
}


void output(set <class NODE, comparator> &st){
  if (st.size() == 0) cout << "EMPTY" << "\n";
  for (auto it = st.begin(); it != st.end(); ++it)
    {
        cout << "(" << (*it).x_coordinate << ", " << (*it).y_coordinate << ") ";
    }
      cout << endl << endl;
}

void output(unordered_map <int, class NODE> &um){
  if (um.size() == 0) cout << "EMPTY" << "\n";
  for (auto it = um.begin(); it != um.end(); ++it)
    {
        cout << "(" << (*it).second.x_coordinate << ", " << (*it).second.y_coordinate << ") ";
    }
    cout << endl << endl;
}


list <NODE> get_successors (NODE *current, vector <vector <bool> > &map, unordered_map <int, class NODE> &CLOSED){
  int x = (*current).x_coordinate;
  int y = (*current).y_coordinate;

  list <NODE> successors;
  NODE tmp;

  for (int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      if (!(map[x-1+i][y-1+j])){
        tmp.x_coordinate = x-1+i;
        tmp.y_coordinate = y-1+j;
        tmp.id_c();
        //add parent
        if(CLOSED.find(tmp.id) == CLOSED.end()) {
          tmp.h_calc(n_goal);
          tmp.g_calc(*current);
          tmp.f_calc();
          successors.push_back(tmp);
        }
      }
    }
  }
  return successors;
}


void update_open(set <class NODE, comparator> &st_open, unordered_set<int> &open_id, NODE &new_node, NODE* pred){
  auto open_iterator = st_open.begin();

  auto is_inside_open = open_id.find(new_node.id);
  if (is_inside_open != open_id.end()){
    while (((*open_iterator).id != new_node.id) && (open_iterator != st_open.end())){
      ++open_iterator;
    }
  }else open_iterator = st_open.end();


  if(open_iterator != st_open.end()){
    NODE tmp = *open_iterator;
    st_open.erase(open_iterator);

    if(tmp.PRED->g > (*pred).g){
      tmp.PRED = pred;
    }
    tmp.check_g(new_node.g);
    st_open.insert(tmp);
  }else{
    //remove new
    new_node.PRED = new NODE;
    new_node.PRED = pred;
    st_open.insert(new_node);
  }
}



int main(int argc, char **argv) {



  //time_t start, end;
  //time(&start);

  ifstream maze;                                                                //Text file with map
  maze.open("/home/what_is_love/Lab/maps/8/map.txt");
  maze >> ::razmer1 >> ::razmer2;
  maze >> ::n_start.x_coordinate >> ::n_start.y_coordinate >> ::n_goal.x_coordinate >> ::n_goal.y_coordinate;

  ::n_start.id_c(); ::n_start.g = 0;
  ::n_goal.id_c(); ::n_goal.h = 0;

  vector <vector <bool> > map (razmer1, vector <bool> (razmer2));               //Initialization of map
  int x, y, b;                                                                  //
  for (int i = 0; i < razmer1; ++i){                                            // - coordinates and states of other nodes
    for(int j = 0; j < razmer2; ++j){
      maze >> x >> y >> b;
      map[x-1][y-1] = bool(b);
    };
  };
  maze.close();

  set <class NODE, comparator> OPEN;
  set <class NODE>::iterator open_start;

  unordered_set <int> OPEN_ID;

  unordered_map <int, class NODE> CLOSED;
  unordered_map <int, class NODE>::iterator address_of_goal_node;

  NODE* pred;

  stack  <NODE> PATH;

  //HEURISTIC ALGORITHM
  int iteration = 0;

  OPEN.insert(n_start);


  cout << "CLOSED FIRST:" << "  ";
  output(CLOSED);
  cout << "OPENED FIRST:" << "  ";
  output(OPEN);

  NODE successor_node, cur_node;
  list <NODE> successors;
  bool goal_is_not_reached = 1;
  cout << "starting" << endl;

  chrono::steady_clock::time_point start = chrono::steady_clock::now();

  while (goal_is_not_reached) {

    if (equivalent(successor_node, n_goal) || (OPEN.size() == 0)) {
      iteration++;
      goal_is_not_reached = 0;
      CLOSED.insert(make_pair(successor_node.id, successor_node));
      address_of_goal_node = CLOSED.find(successor_node.id);
      continue;
    }

    iteration++;
    open_start = OPEN.begin();
    successor_node = *open_start;
    OPEN.erase(open_start);

    cout << "Curently woking on :  " << successor_node.x_coordinate << "  " << successor_node.y_coordinate << endl;
    CLOSED.insert(make_pair(successor_node.id, successor_node));
    pred = &(CLOSED.find(successor_node.id)->second);


    successors = get_successors(pred, map, CLOSED);
    for (auto suc_it = successors.begin(); suc_it != successors.end(); ++suc_it){
      //cur_node
      cur_node = *suc_it;
      OPEN_ID.insert(cur_node.id);
      update_open(OPEN, OPEN_ID, cur_node, pred);
    }

  }
  //END OF ALGORYTHM
  chrono::steady_clock::time_point end = chrono::steady_clock::now();

  cout << "total iterations: " << iteration << endl << endl;
  //cout << "total closed: ";
  //output(CLOSED);
  //cout << endl;



  //PATHFINDING
  NODE path_node = (*address_of_goal_node).second;
  NODE path_node_next;
  PATH.push(path_node);
  bool path_not_complite = 1;

  //TODO: remove PATH stack, make algorithm write path directly to the file

  while(path_not_complite){
    path_node_next = *(path_node.PRED);
    if (equivalent(path_node_next, n_start)){
      path_not_complite = 0;
      PATH.push(n_start);
      continue;
    }
    else{
      PATH.push(path_node_next);
      path_node = path_node_next;
    }
  };


  cout << "PATH DONE" << endl;
  cout << "total iterations:" << iteration << endl;
  cout << "path length: " << PATH.size() << endl;

  ofstream path;
  path.open("/home/what_is_love/Lab/maps/8/path.txt");
  int x_path, y_path;
  int range = PATH.size();
  for (int i = 0; i < range; ++i){
    x_path = PATH.top().x_coordinate;
    y_path = PATH.top().y_coordinate;
    PATH.pop();
    path <<  x_path << " " << y_path << "\n";
  }
  path.close();

  //time(&end);
  //cout << "time is" << difftime(end, start) << endl;


  cout << "Calculating took "
              << chrono::duration_cast<chrono::milliseconds>(end - start).count()
              << "ms.\n";

	return 0;
};
