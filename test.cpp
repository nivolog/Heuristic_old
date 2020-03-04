
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

using namespace std;

int razmer1, razmer2;
double e = 1.0;

class NODE {                                                                    //Main class in programm
private:
	/* data */

public:
 int x_coordinate;                                                              //X position of NODE
 int y_coordinate;                                                              //Y position of NODE
 bool is_blocked;                                                               //1 if NODE is blocked
 double g;                                                                      //distance to the start NODE
 double h;                                                                      //heuristic distance to the goal NODE
 double f;                                                                      //summ of g and h
 int id;
 NODE();                                                                        //constructor
 NODE *PRED;                                                                    //link to the predecessor




 void id_c(void){
   id = y_coordinate*razmer1 + x_coordinate;
 }

 double c(NODE &predecessor){                                                   //distance between two nodes
	int dx = x_coordinate - predecessor.x_coordinate;
	int dy = y_coordinate - predecessor.y_coordinate;
	int dd = fabs(dx*dy);
	if (dd > 0) return sqrt(2);                                                   //sqrt(2) in case of diagonal transition
	else return 1;                                                                //1 in case of horizontal/vertical transition
 }

 void g_calc(NODE &predecessor){
	g = predecessor.g + c(predecessor);
 }
/*
 void h_calc(NODE &goal){                                                       //
	 h = e * hypot(abs(x_coordinate - goal.x_coordinate), abs(y_coordinate - goal.y_coordinate));
 }
 */

/*
 void h_calc(NODE &goal){                                                       //
	 h = abs(x_coordinate - goal.x_coordinate) + abs(y_coordinate - goal.y_coordinate) + (sqrt(2) - 2)*fmin(abs(x_coordinate - goal.x_coordinate),abs(y_coordinate - goal.y_coordinate));
   h *= e;
 }
 */

 void h_calc(NODE &goal){                                                       //
   h = e * fmax(abs(x_coordinate - goal.x_coordinate), abs(y_coordinate - goal.y_coordinate));
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
  //PRED = nullptr;
}

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


//void?
//get successors
vector < vector <NODE> > neighbours_check(NODE &n, vector <vector <NODE> > &neighbours , vector <vector <bool> > &map, unordered_map <int, class NODE> &CLOSED){
  int x = n.x_coordinate;
  int y = n.y_coordinate;
  //list
  for (int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      neighbours[i][j].x_coordinate = x-1+i;
      neighbours[i][j].y_coordinate = y-1+j;
      neighbours[i][j].id_c();
      neighbours[i][j].is_blocked = map[x-1+i][y-1+j];
    }
  }
  /*
  for (int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      cout << "(" <<neighbours[i][j].x_coordinate << " " << neighbours[i][j].y_coordinate << ")  ";
    }
    cout << endl;
  }*/

  for (int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      if(CLOSED.find((neighbours[i][j]).id) != CLOSED.end()) neighbours[i][j].is_blocked = 1;
      //if(CLOSED.find((neighbours[i][j]).id) != CLOSED.end()) cout << "found one! this dude is: (" << neighbours[i][j].x_coordinate << "  " << neighbours[i][j].y_coordinate << ")" << endl;

    }
  }
  return neighbours;
}


set <class NODE, comparator> update_open(set <class NODE, comparator> &st_open, unordered_set<int> &open_id, NODE &new_node, NODE &n_goal, NODE* pred){
  auto open_iterator = st_open.begin();

  auto is_inside_open = open_id.find(new_node.id);
  if (is_inside_open != open_id.end()){
    while (((*open_iterator).id != new_node.id) && (open_iterator != st_open.end())){
      ++open_iterator;
    }
  }else open_iterator = st_open.end();


  if(open_iterator != st_open.end()){
    NODE tmp = *open_iterator;
    cout << "ouch see this one before: (" << tmp.x_coordinate << " " << tmp.y_coordinate << ")" << endl;
    st_open.erase(open_iterator);

    if(tmp.PRED->g > (*pred).g){
      tmp.PRED = pred;
    }
    tmp.check_g((*pred).g + tmp.c(*pred));
    st_open.insert(tmp);
  }else{
    //cout << "umm this is yammy new one: (" << new_node.x_coordinate << " " << new_node.y_coordinate << ")" << endl;
    new_node.PRED = new NODE;
    new_node.PRED = pred;
    new_node.g_calc(*pred);
    new_node.h_calc(n_goal);
    new_node.f_calc();
    st_open.insert(new_node);
  }

  return st_open;
}



int main(int argc, char **argv) {
  time_t start, end;
  time(&start);

  ifstream maze;                                                                //Text file with map
  maze.open("/home/what_is_love/Lab/maps/8/map.txt");                           //Structure is:
  maze >> ::razmer1 >> ::razmer2;                                               //- two numbers, defining size
  NODE n_start, n_goal;                                                         // - cordinates of start and goal nodes
  maze >> n_start.x_coordinate >> n_start.y_coordinate >> n_goal.x_coordinate >> n_goal.y_coordinate;
  n_start.id_c(); n_start.g = 0;
  n_goal.id_c(); n_goal.h = 0;

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
  unordered_map <int, class NODE>::iterator closed_start;
  unordered_map <int, class NODE>::iterator address_of_goal_node;

  NODE* pred;

  stack  <NODE> PATH;

  //HEURISTIC ALGHORITM
  int iteration = 0;

  OPEN.insert(n_start);


  cout << "CLOSED FIRST:" << "  ";
  output(CLOSED);
  cout << "OPENED FIRST:" << "  ";
  output(OPEN);

  NODE s,n;
  vector <vector <NODE> >  neighbours (3, vector <NODE> (3));
  bool goal_is_not_reached = 1;
  cout << "starting" << endl;

  while (goal_is_not_reached) {
    iteration++;
    open_start = OPEN.begin();
    s = *open_start;
    OPEN.erase(open_start);


    cout << "Curently woking on :  " << s.x_coordinate << "  " << s.y_coordinate << endl;

    //найден ли путь?
    if (equivalent(s, n_goal)) {
      iteration++;
      goal_is_not_reached = 0;
      CLOSED.insert(make_pair(s.id, s));
      address_of_goal_node = CLOSED.find(s.id);
      continue;
    }

    /*
    cout << "CLOSED:  ";
    output(CLOSED);
    cout << "OPENED:  ";
    output(OPEN);
    */

    CLOSED.insert(make_pair(s.id, s));

    neighbours = neighbours_check(s, neighbours, map, CLOSED);
    pred = &(CLOSED.find(s.id)->second);

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j)	{
        n = neighbours[i][j];
        OPEN_ID.insert(n.id);
        if (!n.is_blocked){
          OPEN = update_open(OPEN, OPEN_ID, n, n_goal, pred);
          //cout << "Open size;" << OPEN.size() * sizeof(NODE) << "\n";
          //cout << "Close size;" << CLOSED.size() * sizeof(NODE) << "\n";
        }
      }
    }
  }

  cout << "total iterations: " << iteration << endl << endl;

  cout << "total closed: ";
  output(CLOSED);
  cout << endl;



  //PATHFINDING
  NODE p = (*address_of_goal_node).second;
  NODE p_next;
  PATH.push(p);
  bool path_not_complite = 1;

  //TODO: remove PATH stack, make alghoritm write path directly to the file

  while(path_not_complite){
    p_next = *(p.PRED);
    if (equivalent(p_next, n_start)){
      path_not_complite = 0;
      PATH.push(n_start);
      continue;
    }
    else{
      PATH.push(p_next);
      p = p_next;
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

  time(&end);
  cout << "time is" << difftime(end, start) << endl;


	return 0;
};
