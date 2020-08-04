
#include <queue>
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <stack>
#include <algorithm>
#include <typeinfo>
#include <ctime>

using namespace std;

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
 NODE();                                                                        //constructor
 NODE *PRED;                                                                    //link to the predecessor

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

 void h_calc(NODE &goal){                                                       //
	 h = hypot(abs(x_coordinate - goal.x_coordinate), abs(y_coordinate - goal.y_coordinate));
 }


/*
 void h_calc(NODE &goal){                                                       //
	 h = abs(x_coordinate - goal.x_coordinate) + abs(y_coordinate - goal.y_coordinate) + (sqrt(2) - 2)*fmin(abs(x_coordinate - goal.x_coordinate),abs(y_coordinate - goal.y_coordinate));
 }
 */

 /*
 void h_calc(NODE &goal){                                                       //
   h = 0.8 * (abs(x_coordinate - goal.x_coordinate) + abs(y_coordinate - goal.y_coordinate));
 }*/


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



bool cmp_by_f(const NODE &one, const NODE &two){
  return one.f < two.f;
}

bool cmp_by_g(const NODE &one, const NODE &two){
  return one.g > two.g;
}

bool equivalent(NODE &one, NODE &two) {                                         //Test for two nodes being same (used for goal and start)
  if ((one.x_coordinate == two.x_coordinate)&&(one.y_coordinate == two.y_coordinate)) return true;
  else return false;
}



bool not_inside(deque <NODE> &deq, NODE &n){
  for (int i = 0; i < deq.size(); ++i){
    if (equivalent(n, deq[i])) return 0;
  }
  return 1;
}

int find_node(deque <NODE> &vec, NODE &n){
  for (int i = 0; i < vec.size(); ++i){
    if (equivalent(n, vec[i])) return i;
  }
  return 0;
}


void output(deque <NODE> &que){
  if (que.size() == 0) cout << "EMPTY" << "\n";
  for (int i = 0; i < que.size(); ++i){
    cout << "(" << que[i].x_coordinate << ", " << que[i].y_coordinate << ") ";
  }
  cout << "\n";
}

void output(vector <NODE> &vec){
  if (vec.size() == 0) cout << "EMPTY" << "\n";
  for (int i = 0; i < vec.size(); ++i){
    cout << "(" << (vec[i]).x_coordinate << ", " << (vec[i]).y_coordinate << ") ";
  }
  cout << "\n";
}

vector < vector <NODE> > neighbours_check(NODE &n, vector <vector <NODE> > &neighbours , vector <vector <bool> > &map, deque <NODE> &CLOSED){
  int x = n.x_coordinate;
  int y = n.y_coordinate;
  for (int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      neighbours[i][j].x_coordinate = x-1+i;
      neighbours[i][j].y_coordinate = y-1+j;
      neighbours[i][j].is_blocked = map[x-1+i][y-1+j];
    }
  }
  for (int k = 0; k < CLOSED.size(); ++k){
    for (int i = 0; i < 3; ++i){
      for(int j = 0; j < 3; ++j){
        if(equivalent(neighbours[i][j], CLOSED[k])) neighbours[i][j].is_blocked = 1;
      }
    }
  }
  return neighbours;
}


deque <NODE> update_open(deque <NODE> &que, NODE &new_node, NODE &pred, NODE &n_goal){
  if (!not_inside(que, new_node)){
    int index = find_node(que, new_node);
    NODE tmp = que[index];

    if(tmp.PRED->g > pred.g){
      *tmp.PRED = pred;
    }


    tmp.check_g(pred.g + tmp.c(pred));
    que.push_back(tmp);
    que.erase(que.begin() + index);
  }else{
    new_node.PRED = new NODE;
    *new_node.PRED = pred;

    new_node.g_calc(pred);
    new_node.h_calc(n_goal);
    new_node.f_calc();
    que.push_back(new_node);
  }
  return que;
}



int main(int argc, char **argv) {
  time_t start, end;
  time(&start);

  ifstream maze;                                                                //Text file with map
  maze.open("/home/what_is_love/Lab/maps/8/map.txt");                                                         //Structure is:
  int razmer1, razmer2;                                                         // - two numbers, defining size
  maze >> razmer1 >> razmer2;                                                   //
  NODE n_start, n_goal;                                                         // - cordinates of start and goal nodes
  maze >> n_start.x_coordinate >> n_start.y_coordinate >> n_goal.x_coordinate >> n_goal.y_coordinate;
  vector <vector <bool> > map (razmer1, vector <bool> (razmer2));               //Initialization of map
  int x, y, b;                                                                  //NOTE: too lazy to write long names, that's x,y coordinates and status of node
  for (int i = 0; i < razmer1; ++i){                                            // - coordinates and states of other nodes
    for(int j = 0; j < razmer2; ++j){
      maze >> x >> y >> b;
      map[x-1][y-1] = bool(b);
    };
  };
  maze.close();

  deque  <NODE> OPEN;
  deque  <NODE> CLOSED;
  stack  <NODE> PATH;

	//HEURISTIC ALGHORITM
  int iteration = 0;
  OPEN.push_front(n_start);
  n_start.g = 0;

  /*
  cout << "CLOSED FIRST:" << "  ";
  output(CLOSED);
  cout << "OPENED FIRST:" << "  ";
  output(OPEN);
  */

  NODE s,n;
  vector <vector <NODE> >  neighbours (3, vector <NODE> (3));
  bool goal_is_not_reached = 1;
  cout << "starting" << endl;

  while (goal_is_not_reached) {
    iteration++;
    s = OPEN.front();
    OPEN.pop_front();
    cout << "Curently woking on :  ";
    cout << s.x_coordinate << "  " << s.y_coordinate << "\n";
    if (equivalent(s, n_goal)) {
      iteration++;
      goal_is_not_reached = 0;
      CLOSED.push_front(s);
      continue;
    }
    /*
    cout << "CLOSED:  ";
    output(CLOSED);
    cout << "OPENED:  ";
    output(OPEN);
    */
    CLOSED.push_front(s);
    neighbours = neighbours_check(s, neighbours, map, CLOSED);
    /*
    cout << "Neighbours are: ";
    for (int i = 0; i < 3; ++i){
      output(neighbours[i]);
    }
    */

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j)	{
				n = neighbours[i][j];

        if (!n.is_blocked){
          OPEN = update_open(OPEN, n, s, n_goal);
          //cout << "Open size;" << OPEN.size() * sizeof(NODE) << "\n";
          //cout << "Close size;" << CLOSED.size() * sizeof(NODE) << "\n";
        }
      }
    }
    sort(OPEN.begin(), OPEN.end(), cmp_by_f);
  }

  cout << "total closed: ";
  output(CLOSED);




  //PATHFINDING
  NODE p = CLOSED.front();
  CLOSED.pop_front();
  NODE p_next;
  PATH.push(p);
  bool path_not_complite = 1;

  //TODO: remove PATH stack, make alghoritm write path directly to the file

  while(path_not_complite){
    p_next = *p.PRED;
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
  //path << "x y\n";
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
