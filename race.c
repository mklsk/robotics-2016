#include "simpletools.h"  
#include "abdrive.h" 
#include "ping.h"
//#include <string.h>
//#include <stdlib.h>
//#include <limits.h>

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
  
  printf("Adjusting triangle, using values %d and %d...\n", x1, x2);
  
	int temp_diff = x1 - x2;
	double cosin = (double) temp_diff / (double) cell_size_cm;
	printf("Cos: %f\n", cosin);
	double radians = PI / 2.0 - acos(cosin);
	printf("Difference: %d\n", temp_diff);
	printf("Rotating: %f\n", radians);
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
	printf("Adjusting using one wall...\n");
	printf("Adjusting %d ticks in direction %c . . .\n", ticks, direction);
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

	// if((cells[i].west && cells[i].south) || (cells[i].north && cells[i].south)) {
	// 	if(cells[i].west && cells[i].south) {
	// 		swap_direction('e');
	// 	}
	// 	else {
	// 		swap_direction('n');
	// 		adjust_angle(&north_weight);
	// 	}
	// } else {
	// 	swap_direction(adj_dir);
	// 	adjust_angle(adj_weight);
	// }

	/*printf("Weight NESW: %d %d %d %d\n", north_weight, east_weight, south_weight, west_weight);
	printf("X Difference: %d\n", x_diff);
	printf("X Sum: %d\n", x_sum);
	printf("Y Difference: %d\n", y_diff);
	printf("Y Sum: %d\n", y_sum);*/

	switch(direction) {
		case 's':
			//printf("Inverting Y difference!\n");
			y_diff = -1 * y_diff;
			break;
		case 'w':
			//printf("Inverting X difference!\n");
			x_diff = -1 * x_diff;
			break;
	}

	//printf("Current direction is '%c', beginning adjustment . . .\n", direction);

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
	//printf("Adjusting %d ticks in direction %c . . .\n", ticks, direction);
	printf("Average weight from wall: %d\n", avg_weight);
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

void print_cells() //print data of all cells
{
	int i;

	for (i=0; i < 16; i++)
	{
		printf("id - %d x - %d y - %d n,s,e,w: %d%d%d%d visited - %d type - %c\n", i,cells[i].x, cells[i].y, 
			cells[i].north, cells[i].south,cells[i].east, cells[i].west, cells[i].visited, cells[i].type);
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

	printf("I can't go %c, as I came from there\n", cant);

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
		printf("n is available\n");
	}
	if (w_av == 1)
	{
		dest = 'w';
		printf("w is available\n");
	}
	if (s_av == 1)
	{
		dest = 's';
		printf("s is available\n");
	}
	if (e_av == 1)
	{
		dest = 'e';
		printf("e is available\n");
	}

	printf("Randomly choose %c\n", dest);

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

	printf("Accessable neighbour with least marks is %c\n", min_dir);

	swap_direction(min_dir);	
}

void cell_analysis() //determine type of cell
{
	int i = find_current_cell();

		if(cells[i].type == 'p') //if path  - just follow it
		{
			printf("Path\n");
			choose_direction_p();
		}

		if(cells[i].type == 'd') // if dead end - turn 180
		{
			printf("Dead end\n");
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
					printf("New path, new junction\n");
					choose_direct_rand_j(); //select a random direction
				}

				else //if visited junction before
				{
					printf("New path, visited junction\n");
					rotate_180();
				}
			}

			else // if coming from an already visited path
			{
				printf("Visited path, some junction\n");
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

	printf("I'm done, master!\n");
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
		printf("\nFor %d:\n", i);
		n_can = 0;
		s_can = 0;
		e_can = 0;
		w_can = 0;

		if(cells[i].visited == 0)
		{
			printf("That cell wasn't visited!\n");
			continue;
		}

		if(cells[i].y < 4) //check if cell on the north is possible
		{
			x = cells[i].x;
			y = cells[i].y + 1;
			n = find_cell(x,y); //get cell on the north
			printf("	Theoretically can connect to %d\n", n);
			if(cells[n].visited > 0)
			{
			n_can = 1;
			printf("	%d is visited, can connect\n", n);
			}
		}

		if(cells[i].y > 1) //check if cell on the south is possible
		{
			x = cells[i].x;
			y = cells[i].y - 1;
			s = find_cell(x,y); //get cell on the south
			printf("	Theoretically can connect to %d\n", s);
			if(cells[s].visited > 0)
			{
				s_can = 1;
				printf("	%d is visited, can connect\n", s);
			}
		}

		if(cells[i].x < 4) //check if cell on the east is possible
		{
			x = cells[i].x + 1;
			y = cells[i].y;
			e = find_cell(x,y); //get cell on the east
			printf("	Theoretically can connect to %d\n", e);
			if(cells[e].visited > 0)
			{
				e_can = 1;
				printf("	%d is visited, can connect\n", e);
			}
		}

		if(cells[i].x > 1) //check if cell on the south is possible
		{
			x = cells[i].x - 1;
			y = cells[i].y;
			w = find_cell(x,y); //get cell on the south
			printf("	Theoretically can connect to %d\n", w);
			if(cells[w].visited > 0)
			{
				w_can = 1;
				printf("	%d is visited, can connect\n", w);
			}

		}


		if(cells[i].north == 0 &&  n_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][n] = 1;
			cost_matrix[n][i] = 1;
			printf("	Cell %d is connected to %d\n", i,n);
		}

		if(cells[i].south == 0 && s_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][s] = 1;
			cost_matrix[s][i] = 1;
			printf("	Cell %d is connected to %d\n", i,s);
		}

		if(cells[i].east == 0 && e_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][e] = 1;
			cost_matrix[e][i] = 1;
			printf("	Cell %d is connected to %d\n", i,e);
		}

		if(cells[i].west == 0 && w_can == 1) //if no wall and cell was visited, write to table
		{
			cost_matrix[i][w] = 1;
			cost_matrix[w][i] = 1;
			printf("	Cell %d is connected to %d\n", i,w);
		}
	}
}

void print_cost_matrix()
{
	int i, j;

	printf("\n     ");
	for(i = 0; i < 16; i++)
	{
		printf("%2d  ", i);
	}

	printf("\n____________________________________________________________________\n\n");

	for (i=0; i<16; i++)
	{ 

	printf("%2d | ",i);

    for(j=0; j<16; j++)
         printf("%2d  ", cost_matrix[i][j]);

    printf("\n");
	 }

	printf("\n");
}

/*void print_pass_matrix()
{
	int i, j;

	printf("\n     ");
	for(i = 0; i < 16; i++)
	{
		printf("%2d  ", i);
	}

	printf("\n____________________________________________________________________\n\n");

	for (i=0; i<16; i++)
	{ 

	printf("%2d | ",i);

    for(j=0; j<16; j++)
         printf("%2d  ", pass_matrix[i][j]);

    printf("\n");
	 }

	printf("\n");
}

void floyds()
{
	int k, i,j;

	for (k = 0; k < 16; k++)
	{
		for (i = 0; i < 16; i++)
		{
			for (j = 0; j < 16; j++)
			{
				if (cost_matrix[i][k] + cost_matrix[k][j] < cost_matrix[i][j])
				{
					cost_matrix[i][j] = cost_matrix[i][k] + cost_matrix[k][j];
					pass_matrix[i][j] = k;
				}
				
			}
			
		}

	}
}*/

void path_init()
{
	int i;
	for (i = 0; i < 16; i++)
	{
		short_path[i] = -1;
	}
}

/*int* char_to_int(char* src)
{
	int l = strlen(src);
	int* arr = malloc (sizeof(int)*l);

	int x;


	for (x = 0; x < l; x++)
	{
		arr[x] = src[x] - '0';
	}

	free(src);
	
	return arr;

}

char* append(char* one, char* two)
{
	int one_l = strlen(one);
	int two_l = strlen(two);

	char* arr = malloc (sizeof(char) * (one_l + two_l));
	strcat(arr, one);
	strcat(arr, two);

	free(one);
	free(two);

	return arr;
}

char* shortest_path(int i, int j)
{
	int r,k;

	if (cost_matrix[i][j] == 99)
	{
		char* arr = malloc(sizeof(char)*0);
		printf("empty arr\n");
		return arr;
	}

	k = pass_matrix[i][j];

	if (k == -1)
	{
		char* arr = malloc(sizeof(char)*1);
		*arr = j;
		printf("single node %d\n", j);
		return arr;	
	}

	else
	{
		printf("good\n");
		return append(shortest_path(i, k), shortest_path(k, j));

	}
}*/

void follow_shortest(int do_adjustment) {

	printf("Following the shortest path:\n");

	int i;
	for (i = 0; i < 16; i++)
	{
		printf("%d ", short_path[i]);
	}

	printf("\n\n");

	int k = 0;
	while(short_path[k] != -1) {
		printf("Considering node #%d in the path...\n", k);
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

		if(cell.x != current_x) {
			if(cell.x > current_x) {
				printf("Moving east...");
				swap_direction('e');
				move_forward();
				current_x++;
			} else {
				printf("Moving west...");
				swap_direction('w');
				move_forward();
				current_x--;
			}
		} else if(cell.y != current_y) {
			if(cell.y > current_y) {
				printf("Moving north...");
				swap_direction('n');
				move_forward();
				current_y++;
			} else {
				printf("Moving south...");
				swap_direction('s');
				move_forward();
				current_y--;
			}
		}
		
		printf("\n");
		
		k++;
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
	printf("X1 = %d\n", temp_dist);
	swap_direction(check_dir);
	move_forward();
	swap_direction(wall_dir);
	int temp_dist_2 = pingDistance();
	printf("X2 = %d\n", temp_dist_2);
	double temp_diff = temp_dist - temp_dist_2;
	double cosin = (double) temp_diff / (double) cell_size_cm;
	printf("Cos: %f\n", cosin);
	double radians = PI / 2.0 - acos(cosin);
	printf("Difference: %lf\n", temp_diff);
	printf("Rotating: %f\n", radians);
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
			printf("\nDistance of %d = %d", i, distance[i]);
			printf("\nPath = %d", i);
			int count = 1;
			int temp[16];
			temp[0] = i;
			j=i;

			do
			{
				j=pred[j];
				printf(" <-%d", j);
				temp[count] = j;
				count++; 
			}
			while(j!=startnode);

			printf("\n");

			int x = 0;

			for(x = 0; x < count; x++)
			{
				printf("%d ", temp[x]);
			}
			printf(" - temp\n");

			x = 0;
			int y;
			count--;
			print("Count - %d\n", count);

			for(y = count; y >= 0; y--)
			{
				short_path[x] = temp[count];
				printf("%d\n", short_path[x]);
				count--;
				x++;
			}


		}
}

int main (void)
{

	printf("Race Algorithm v0.8.7\n\n");

	initialise_cells(); // initiaise cells in the map with indices and false for all walls

	tremaux();

	convert_unknown_to_walls();

	create_matrices();
	print_cost_matrix();

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
