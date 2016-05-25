#include "simpletools.h"  
#include "abdrive.h" 
#include "ping.h"

#define INFINITY 9999
#define MAX 16
#define ABW 106
#define DPT 3.25
#define PI 3.14159265358979323846


typedef struct Cell {
   int x;
   int y;
   int north;
   int south;
   int east;
   int west;
   int visited;
   char type;      
} Cell;

Cell cells [16]; //array of cells

int cell_size_cm = 40;
int cell_size_ticks = DPT * 40;

int cost_matrix[16][16];
int pass_matrix[16][16];
int short_path[16];

char direction = 'n'; 

int current_x = 1;
int current_y = 1;

int prev_x = 1;
int prev_y = 1;

int goal1_x = 4; //coordinates of point that need to be reached
int goal1_y = 4;
int goal2_x = 4;
int goal2_y = 1;
int goal3_x = 1;
int goal3_y = 4;

int north_weight;
int south_weight;
int east_weight;
int west_weight;

int prev_north_weight;
int prev_south_weight;
int prev_east_weight;
int prev_west_weight;

int avg_weight = -1;

int checked_walls = 0;

void rotateZeroRadius(double radians) {
  //double radians = PI * (double) angle / 180.0;
  double distancePerWheel = radians * ABW / 2;
  int ticksPerWheel = distancePerWheel / DPT;
  drive_goto(ticksPerWheel, -ticksPerWheel);
  //printf("Ticks rotation: %d\n", ticksPerWheel);
}

void update_position() //update robots current position
{

	prev_x = current_x;
	prev_y = current_y;

	switch (direction)
		{
			case 'n' :
			current_y++;
			break;

			case 'w' :
			current_x--;
			break;

			case 's' :
			current_y--;
			break;

			case 'e' :
			current_x++;
			break;
		}

		//printf("x - %d, y - %d\n", current_x, current_y);
		//printf("x - %d, y - %d\n", prev_x, prev_y);
}

int find_cell(int x, int y) //find a particular cell
{

	int i;

	for(i = 0; i<16; i++) //find current cell
	{
		if(cells[i].x == x && cells[i].y == y)
			break;
	}

	return i;
}

int find_current_cell() //find, which cell does the robot stand now on
{

	int i;

	for(i = 0; i<16; i++) //find current cell
	{
		if(cells[i].x == current_x && cells[i].y == current_y)
			break;
	}

	return i;
}

int find_prev_cell() //find, which cell does the robot stand now on
{

	int i;

	for(i = 0; i<16; i++) //find current cell
	{
		if(cells[i].x == prev_x && cells[i].y == prev_y)
			break;
	}

	return i;
}

void update_visited() //update data about whether a cell was visited or not
{
	int i = find_current_cell();

	cells[i].visited++;
}

int pingDistance() //measure distance from a wall in current direction
{

  return ping_cm(8);;
}

void move_forward () //move one cell forward in current direction
{

	drive_goto(cell_size_ticks,cell_size_ticks);
}

void turn (char dir) //turn clockwise or anti-clockwise
{
	int left, right;

	switch (dir)
	{

		case 'a' : 
		//rotateZeroRadius(-PI/2);
		drive_goto(-26,25);

		switch (direction)
		{
			case 'n' :
			direction = 'w';
			break;

			case 'w' :
			direction = 's';
			break;

			case 's' :
			direction = 'e';
			break;

			case 'e' :
			direction = 'n';
			break;
		}


		break;

		case 'c' : 
		//rotateZeroRadius(PI/2);
		drive_goto(25,-26);
		switch (direction)
		{
			case 'n' :
			direction = 'e';
			break;

			case 'w' :
			direction = 'n';
			break;

			case 's' :
			direction = 'w';
			break;

			case 'e' :
			direction = 's';
			break;
		}

		break;
	}
}

void ping_wall(int i) //check current side for walls
{
	if (1)//pingDistance() < 25) //if closer than 25 - write that there is a wall
	{
		switch (direction)
		{
			case 'n' :
				if(pingDistance() < 30) 
				{
					cells [i].north = 1;
					//printf("wall on n\n");
				} else {
					cells [i].north = 0;
				}
				north_weight = pingDistance(); //save distance from north wall
				break;

			case 'w' :
				if(pingDistance() < 30) 
				{
					cells [i].west = 1;
					//printf("wall on w\n");
				} else {
					cells [i].west = 0;
				}
				west_weight = pingDistance(); //save distance from north wall
				break;

			case 's' :
				if(pingDistance() < 30) 
					{
						cells [i].south = 1;
						//printf("wall on s\n");
					} else {
						cells [i].south = 0;
					}
				south_weight = pingDistance(); //save distance from north wall
				break;

			case 'e' :
				if(pingDistance() < 30) 
					{
						cells [i].east = 1;
						//printf("wall on e\n");
					} else {
						cells [i].east = 0;
					}
				east_weight = pingDistance(); //save distance from north wall
				break;

		}
	}
}

void adjust_angle_triangle(int x1, int x2) {
  
  if(prev_x == current_x && prev_y == current_y) return;

  
	int temp_diff = x1 - x2;
	double cosin = (double) temp_diff / (double) cell_size_cm;

	double radians = PI / 2.0 - acos(cosin);

	rotateZeroRadius(radians);
}

void swap_direction(char goal) //turn robot to a desired direction
{
	if(goal != direction)
	{

		switch (goal)
		{
			case ('n'): //if need to turn to north
			switch (direction)
			{
				case ('s'):
				turn('c');
				turn('c');
				break;

				case ('e'):
				turn('a');
				break;

				case ('w'):
				turn('c');
				break;
			}
			break;

			case ('s'):
			switch (direction)
			{
				case ('n'):
				turn('c');
				turn('c');
				break;

				case ('e'):
				turn('c');
				break;

				case ('w'):
				turn('a');
				break;
			}
			break;

			case ('e'):
			switch (direction)
			{
				case ('n'):
				turn('c');
				break;

				case ('s'):
				turn('a');
				break;

				case ('w'):
				turn('c');
				turn('c');
				break;
			}
			break;

			case ('w'):
			switch (direction)
			{
				case ('n'):
				turn('a');
				break;

				case ('s'):
				turn('c');
				break;

				case ('e'):
				turn('c');
				turn('c');
				break;
			}
			break;


		}
	}
}

void adjust_one_wall(int *temp_weight) {
	ping_wall(find_current_cell());
	int difference = *temp_weight - avg_weight;
	int ticks = difference * 10.0 / 3.25;
	drive_goto(ticks, ticks);
	*temp_weight = avg_weight;
}

void check_wall_weights() {

	int i = find_current_cell();

	prev_north_weight = north_weight;
	prev_east_weight = east_weight;
	prev_south_weight = south_weight;
	prev_west_weight = west_weight;

  switch(direction) {
    case 'n':
      cells[i].south = 0;
      south_weight = 100;
      break;
    case 'e':
      cells[i].west = 0;
      west_weight = 100;
      break;
    case 's':
      cells[i].north = 0;
      north_weight = 100;
      break;
    case 'w':
      cells[i].east = 0;
      east_weight = 100;
      break;
  }

	ping_wall(i);
	
	
	turn('c');
	ping_wall(i);

	turn('a');
	turn('a');
	ping_wall(i);

	turn('a');
	ping_wall(i);

	turn('c');
	turn('c');
	
	/*
  char memory_direction = direction;

	if(cells[i].north != 0) {
		print("North unknown, checking...\n");
		swap_direction('n');
		ping_wall(i);
	}
	if(cells[i].east != 0) {
		print("East unknown, checking...\n");
		swap_direction('e');
		ping_wall(i);
	}
	if(cells[i].south != 0) {
		print("South unknown, checking...\n");
		swap_direction('s');
		ping_wall(i);
	}
	if(cells[i].west != 0) {
		print("West unknown, checking...\n");
		swap_direction('w');
		ping_wall(i);
	}
	
	swap_direction(memory_direction);*/

}

void check_walls () //check all sides for walls
{

	check_wall_weights();

	int i = find_current_cell();
	int z = find_prev_cell();

	int memory_direction = direction;

	int y_diff = north_weight - south_weight;
	int y_sum = north_weight + south_weight;
	int x_diff = east_weight - west_weight;
	int x_sum = east_weight + west_weight;

	int diff_treshold = 18;
	int sum_treshold = 60;

	char adj_dir_1 = '\0';
	int *adj_weight_1;
	char adj_dir_2 = '\0';
	int *adj_weight_2;
	if(cells[i].north) {
		adj_dir_1 = 'n';
		adj_weight_1 = &north_weight;
	} else if(cells[i].south) {
		adj_dir_1 = 's';
		adj_weight_1 = &south_weight;
	}
	if(cells[i].east) {
		adj_dir_2 = 'e';
		adj_weight_2 = &east_weight;
	} else if(cells[i].west) {
		adj_dir_2 = 'w';
		adj_weight_2 = &west_weight;
	}


	int adj_weight_prev;
	int adj_weight_next;
	int do_adj = 0;
	if(cells[i].north == 1 && cells[z].north == 1) {
		adj_weight_next = north_weight;
		adj_weight_prev = prev_north_weight;
		do_adj = 1;
	} else if(cells[i].east == 1 && cells[z].east == 1) {
		adj_weight_next = -east_weight;
		adj_weight_prev = -prev_east_weight;
		do_adj = 1;
	} else if(cells[i].south == 1 && cells[z].south == 1) {
		adj_weight_next = -south_weight;
		adj_weight_prev = -prev_south_weight;
		do_adj = 1;
	} else if(cells[i].west == 1 && cells[z].west == 1) {
		adj_weight_next = west_weight;
		adj_weight_prev = prev_west_weight;
		do_adj = 1;
	}

  if(do_adj && checked_walls) adjust_angle_triangle(adj_weight_prev, adj_weight_next);

	

	switch(direction) {
		case 's':
			y_diff = -1 * y_diff;
			break;
		case 'w':
			x_diff = -1 * x_diff;
			break;
	}


	double adjustment = 0;

	if(abs(y_diff) < diff_treshold && y_sum < sum_treshold) {
		if(direction != 'n' && direction != 's') swap_direction('n');
		adjustment = (double) y_diff / 2.0;
		north_weight = y_sum / 2;
		south_weight = y_sum / 2;
		if(avg_weight == -1) avg_weight = y_sum / 2;
		else {
			avg_weight += y_sum / 2;
			avg_weight /= 2;
		}
		
	} else if(abs(x_diff) < diff_treshold && x_sum < sum_treshold) {
		if(direction != 'e' && direction != 'w') swap_direction('e');
		adjustment = (double) x_diff / 2.0;
		west_weight = x_sum / 2;
		east_weight = x_sum / 2;
		if(avg_weight == -1) avg_weight = x_sum / 2;
		else {
			avg_weight += x_sum / 2;
			avg_weight /= 2;
		}
	} else if(avg_weight != -1) {
	  if(adj_dir_1 != '\0') {
  		swap_direction(adj_dir_1);
  		adjust_one_wall(adj_weight_1);
	  }
	  if(adj_dir_2 != '\0') {
  		swap_direction(adj_dir_2);
  		adjust_one_wall(adj_weight_2);
	  }
	}

	int ticks = adjustment * 10.0 / 3.25;
	drive_goto(ticks, ticks);
	swap_direction(memory_direction);

}

void determ_type(int i) //determine if cell is an elem of path, a dead end or junction
{
	int sum = cells[i].north + cells[i].south + cells[i].east + cells[i].west;

	if (sum == 3)
		cells[i].type = 'd';
	if (sum == 2)
		cells[i].type = 'p';
	if (sum < 2)
		cells[i].type = 'j';

	//printf("Type - %c \n", cells[i].type);
}

void move () //set of actions performed each time the robot moves
{
	
	move_forward();
	update_position();

	int i = find_current_cell();

	if (cells[i].visited == 0)
	{
		check_walls();
		determ_type(i);
		checked_walls = 1;
	}

	else
	{
		checked_walls = 0;
	}


	update_visited();
}

void initialise_cells() //give index and other basic value to cells
{
	int i;

	for (i = 0; i < 16; i++)
	{
		cells[i].x = i / 4 + 1;
		cells[i].y = i % 4 + 1;

		cells[i].north = -1;
		cells[i].south = -1;
		cells[i].east = -1;
		cells[i].west = -1;

		cells[i].visited = 0;
	}
}


void move_out() //initial move out of 
{
	move_forward(); //roll out
	update_visited();
	check_walls();
	determ_type(0);
}

char cant_go () //find a direction from which robot came, and thus shouldn't go there
{
	char cant_go;

	if (direction == 'n') //find direction from which robot came - can't go there
		cant_go = 's';
	if (direction == 's')
		cant_go = 'n';
	if (direction == 'e')
		cant_go = 'w';
	if (direction == 'w')
		cant_go = 'e';

	return cant_go;
}

void choose_direction_p() //choosing direction for an element of path
{
	char cant = cant_go();
	char will_go;
	int i = find_current_cell();

	//printf("I came from %c\n", cant);

	if(cells[i].north == 0 && cant != 'n') //find direction in which the robot will move
		will_go = 'n';
	if(cells[i].south == 0 && cant != 's')
		will_go = 's';
	if(cells[i].east == 0 && cant != 'e')
		will_go = 'e';
	if(cells[i].west == 0 && cant != 'w')
		will_go = 'w';


	//printf("I will go %c\n", will_go);

	
	swap_direction(will_go); //turn to where you want to go
}

void choose_direct_rand_j() //on a new junction, choose random direction
{
	char cant = cant_go();
	char dest;
	int i = find_current_cell();
	int n_av = 1;
	int s_av = 1;
	int e_av = 1;
	int w_av = 1;

	switch (cant)
	{
		case ('n'):
		n_av = 0;
		break;

		case ('s'):
		s_av = 0;
		break;

		case ('e'):
		e_av = 0;
		break;

		case ('w'):
		w_av = 0;
		break;
	}

	if (cells[i].north == 1)
		n_av = 0;
	if (cells[i].south == 1)
		s_av = 0;
	if (cells[i].east == 1)
		e_av = 0;
	if (cells[i].west == 1)
		w_av = 0;

	

	if (n_av == 1)
	{
		dest = 'n';
	}
	if (w_av == 1)
	{
		dest = 'w';
	}
	if (s_av == 1)
	{
		dest = 's';
	}
	if (e_av == 1)
	{
		dest = 'e';
	}


	swap_direction(dest);
}

void rotate_180() //rotate the robot 180 degrees
{
	switch (direction)
			{
				case ('n'):
				swap_direction('s');
				break;

				case ('s'):
				swap_direction('n');
				break;

				case ('e'):
				swap_direction('w');
				break;

				case ('w'):
				swap_direction('e');
				break;

			}
}

void with_least_marks() //find a neighbour of the cell which was the least visited
{
	int min = 3;
	char min_dir;
	int i = find_current_cell();
	int j;


	if(cells[i].east == 0) //if can enter cell on east
	{
		current_x++; //find cell on east
		j = find_current_cell();
		current_x--;

		if (cells[j].visited < min)
		{
			min = cells[j].visited;
			min_dir = 'e';
		}
	}

	if(cells[i].north == 0) //if can enter cell on north
	{
		current_y++; //find cell on north
		j = find_current_cell();
		current_y--;

		if (cells[j].visited < min)
		{
			min = cells[j].visited;
			min_dir = 'n';
		}
	}

	if(cells[i].west == 0) //if can enter cell on west
	{
		current_x--; //find cell on west
		j = find_current_cell();
		current_x++;

		if (cells[j].visited < min)
		{
			min = cells[j].visited;
			min_dir = 'w';
		}
	}

	if(cells[i].south == 0) //if can enter cell on south
	{
		current_y--; //find cell on south
		j = find_current_cell();
		current_y++;

		if (cells[j].visited < min)
		{
			min = cells[j].visited;
			min_dir = 's';
		}
	}


	swap_direction(min_dir);	
}

void cell_analysis() //determine type of cell
{
	int i = find_current_cell();

		if(cells[i].type == 'p') //if path  - just follow it
		{
			choose_direction_p();
		}

		if(cells[i].type == 'd') // if dead end - turn 180
		{
			rotate_180();
			update_visited();

		}

		if(cells[i].type == 'j') //if junction 
		{
			
			int p = find_prev_cell();

			if (cells[p].visited == 1) //if coming from unvisited
			{
				if (cells[i].visited == 1) //if new junction
				{
					choose_direct_rand_j(); //select a random direction
				}

				else //if visited junction before
				{
					rotate_180();
				}
			}

			else // if coming from an already visited path
			{
				with_least_marks(); //choose option which was the least visited
			}
	
		}		

		//print_cells();
}

void tremaux() //do tremaux algorithm
{
	

	move_forward(); //move out of the initial cell
	check_walls();
	determ_type(0);
	update_visited();

	int k,m,l;

	k = find_cell(goal1_x, goal1_y); //get goal cells
	m = find_cell(goal2_x, goal2_y);
	l = find_cell(goal3_x, goal3_y);


	while(cells[k].visited == 0 || cells[l].visited == 0 || cells[m].visited == 0 )
	{
		cell_analysis();

		int i = find_current_cell();
		if (i == k || i == l || i == m)
			printf("Master I've found him!!! >:3\n");

		move();

	}
}

void create_matrices() //fills up cost and pass matrices with cost values
{
	int i,j;
	int n,s,e,w;
	int n_can, s_can, e_can, w_can;
	int x,y;

	for(i = 0; i<16; i++)
	{
		for(j = 0; j<16; j++)
		{
			if(i == j)
			{
				cost_matrix[i][j] = 0; //no cost of going to itself
			}

			else
			{
			cost_matrix[i][j] = 99; //by default - no connection for all nodes
			}

			pass_matrix[i][j] = -1;
		}
	}

	for (i = 0; i < 16; i++)
	{
		n_can = 0;
		s_can = 0;
		e_can = 0;
		w_can = 0;

		if(cells[i].visited == 0)
		{
			continue;
		}

		if(cells[i].y < 4) //check if cell on the north is possible
		{
			x = cells[i].x;
			y = cells[i].y + 1;
			n = find_cell(x,y); //get cell on the north
			if(cells[n].visited > 0)
			{
			n_can = 1;
			}
		}

		if(cells[i].y > 1) //check if cell on the south is possible
		{
			x = cells[i].x;
			y = cells[i].y - 1;
			s = find_cell(x,y); //get cell on the south
			if(cells[s].visited > 0)
			{
				s_can = 1;
			}
		}

		if(cells[i].x < 4) //check if cell on the east is possible
		{
			x = cells[i].x + 1;
			y = cells[i].y;
			e = find_cell(x,y); //get cell on the east
			if(cells[e].visited > 0)
			{
				e_can = 1;
			}
		}

		if(cells[i].x > 1) //check if cell on the south is possible
		{
			x = cells[i].x - 1;
			y = cells[i].y;
			w = find_cell(x,y); //get cell on the south
			if(cells[w].visited > 0)
			{
				w_can = 1;

			}

		}


		if(cells[i].north == 0 &&  n_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][n] = 1;
			cost_matrix[n][i] = 1;
		}

		if(cells[i].south == 0 && s_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][s] = 1;
			cost_matrix[s][i] = 1;
		}

		if(cells[i].east == 0 && e_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][e] = 1;
			cost_matrix[e][i] = 1;
		}

		if(cells[i].west == 0 && w_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][w] = 1;
			cost_matrix[w][i] = 1;
		}
	}
}



void path_init()
{
	int i;
	for (i = 0; i < 16; i++)
	{
		short_path[i] = -1;
	}
}


void follow_shortest(int do_adjustment) {


	int i;
	for (i = 0; i < 16; i++)
	{
	}


	int k = 0;
	while(short_path[k] != -1) {
		i = find_current_cell();
		int z = find_prev_cell();
		Cell cell = cells[short_path[k]];

		if(do_adjustment) {

      check_wall_weights();

    	int adj_weight_prev;
    	int adj_weight_next;
    	int do_adj = 0;
    	if(cells[i].north == 1 && cells[z].north == 1) {
    		adj_weight_next = north_weight;
    		adj_weight_prev = prev_north_weight;
    		do_adj = 1;
    	} else if(cells[i].east == 1 && cells[z].east == 1) {
    		adj_weight_next = east_weight;
    		adj_weight_prev = prev_east_weight;
    		do_adj = 1;
    	} else if(cells[i].south == 1 && cells[z].south == 1) {
    		adj_weight_next = south_weight;
    		adj_weight_prev = prev_south_weight;
    		do_adj = 1;
    	} else if(cells[i].west == 1 && cells[z].west == 1) {
    		adj_weight_next = west_weight;
    		adj_weight_prev = prev_west_weight;
    		do_adj = 1;
    	}
    
      if(do_adj) adjust_angle_triangle(adj_weight_prev, adj_weight_next);


			// Pick adjustment wall
  	char adj_dir_1 = '\0';
  	int *adj_weight_1;
  	char adj_dir_2 = '\0';
  	int *adj_weight_2;
  	if(cells[i].north) {
  		adj_dir_1 = 'n';
  		adj_weight_1 = &north_weight;
  	} else if(cells[i].south) {
  		adj_dir_1 = 's';
  		adj_weight_1 = &south_weight;
  	}
  	if(cells[i].east) {
  		adj_dir_2 = 'e';
  		adj_weight_2 = &east_weight;
  	} else if(cells[i].west) {
  		adj_dir_2 = 'w';
  		adj_weight_2 = &west_weight;
  	}

			if(adj_dir_1 != '\0') {
				swap_direction(adj_dir_1);
				adjust_one_wall(adj_weight_1);
			}
			if(adj_dir_2 != '\0') {
				swap_direction(adj_dir_2);
				adjust_one_wall(adj_weight_2);
			}

		}
		
		prev_x = current_x;
		prev_y = current_y;


		int j = 1;

		if(cell.x != current_x) {
			if(cell.x > current_x) {
				swap_direction('e');
				if(!do_adjustment) {
					while(short_path[k + j] != -1) {
						int cell_curr_temp = short_path[k + j - 1];
						int cell_next_temp = short_path[k + j];
						if(cells[cell_next_temp].x > cells[cell_curr_temp].x) {
							j++;
						} else {
							break;
						}
					}
				}
				drive_goto(cell_size_ticks * j, cell_size_ticks * j);
				current_x += j;
			} else {
				swap_direction('w');
				if(!do_adjustment) {
					while(short_path[k + j] != -1) {
						int cell_curr_temp = short_path[k + j - 1];
						int cell_next_temp = short_path[k + j];
						if(cells[cell_next_temp].x < cells[cell_curr_temp].x) {
							j++;
						} else {
							break;
						}
					}
				}
				drive_goto(cell_size_ticks * j, cell_size_ticks * j);
				current_x -= j;
			}
		} else if(cell.y != current_y) {
			if(cell.y > current_y) {
				swap_direction('n');
				if(!do_adjustment) {
					while(short_path[k + j] != -1) {
						int cell_curr_temp = short_path[k + j - 1];
						int cell_next_temp = short_path[k + j];
						if(cells[cell_next_temp].y > cells[cell_curr_temp].y) {
							j++;
						} else {
							break;
						}
					}
				}
				drive_goto(cell_size_ticks * j, cell_size_ticks * j);
				current_y += j;
			} else {
				swap_direction('s');
				if(!do_adjustment) {
					while(short_path[k + j] != -1) {
						int cell_curr_temp = short_path[k + j - 1];
						int cell_next_temp = short_path[k + j];
						if(cells[cell_next_temp].y < cells[cell_curr_temp].y) {
							j++;
						} else {
							break;
						}
					}
				}
				drive_goto(cell_size_ticks * j, cell_size_ticks * j);
				current_y -= j;
			}
		}
		
		
		k += j;
	}
}


void convert_unknown_to_walls() {
	int i;
	for (i = 0; i < 16; i++)
	{
		if(cells[i].north == -1) cells[i].north = 0;
		if(cells[i].south == -1) cells[i].south = 0;
		if(cells[i].east == -1) cells[i].east = 0;
		if(cells[i].west == -1) cells[i].west = 0;
	}
}


double acos(double x) {

   return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;
}

void fix_angle(char wall_dir, char check_dir)
{
	swap_direction(wall_dir);
	int temp_dist = pingDistance();
	swap_direction(check_dir);
	move_forward();
	swap_direction(wall_dir);
	int temp_dist_2 = pingDistance();
	double temp_diff = temp_dist - temp_dist_2;
	double cosin = (double) temp_diff / (double) cell_size_cm;
	double radians = PI / 2.0 - acos(cosin);
	rotateZeroRadius(radians);
}

void dijikstra(int G[MAX][MAX], int n, int startnode, int endnode)
{
	int cost[MAX][MAX], distance[MAX], pred[MAX];
	int visited[MAX], count, mindistance, nextnode, i,j;
	for(i=0;i < n;i++)
		for(j=0;j < n;j++)
			if(G[i][j]==0)
				cost[i][j]=INFINITY;
			else
				cost[i][j]=G[i][j];
	
	for(i=0;i< n;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}
	distance[startnode]=0;
	visited[startnode]=1;
	count=1;
	while(count < n-1){
		mindistance=INFINITY;
		for(i=0;i < n;i++)
			if(distance[i] < mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}
		visited[nextnode]=1;
		for(i=0;i < n;i++)
			if(!visited[i])
				if(mindistance+cost[nextnode][i] < distance[i])
				{
					distance[i]=mindistance+cost[nextnode][i];
					pred[i]=nextnode;
				}
			count++;
	}
 
	for(i=0;i < n;i++)
		if(i == endnode)
		{
			int count = 1;
			int temp[16];
			temp[0] = i;
			j=i;

			do
			{
				j=pred[j];
				temp[count] = j;
				count++; 
			}
			while(j!=startnode);

			int x = 0;

			

			x = 0;
			int y;
			count--;
			

			for(y = count; y >= 0; y--)
			{
				short_path[x] = temp[count];
				count--;
				x++;
			}


		}
}

int main (void)
{

	

	initialise_cells(); // initiaise cells in the map with indices and false for all walls

	tremaux();

	convert_unknown_to_walls();

	create_matrices();

	path_init();

	dijikstra(cost_matrix,16,find_current_cell(),0);
	follow_shortest(1);

	swap_direction('s');
	move_forward();
	swap_direction('n');

	//blink light

	move_forward();
	path_init();
	dijikstra(cost_matrix,16,0,15);
	follow_shortest(0);

	//blink light





	return 0;
}
