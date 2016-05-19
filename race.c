#include "simpletools.h"  
#include "abdrive.h" 
#include "ping.h"

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

Cell cells [16];
char direction = 'n'; 

int current_x = 1;
int current_y = 1;
int prev_x = 1;
int prev_y = 1;
int goal_x = 4;
int goal_y = 4;

int north_weight;
int south_weight;
int east_weight;
int west_weight;


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

  return ping_cm(8);
}

void move_forward () //move one cell forward in current direction
{

	int dist = 123;
	drive_goto(dist,dist);
}

void turn (char dir) //turn clockwise or anti-clockwise
{
	int left, right;

	switch (dir)
	{

		case 'a' : 
		drive_goto(-25,26);

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
		drive_goto(26,-25);

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

void adjust_ping ()
{

	int new = pingDistance();
	int dir = 1; //initially try clockwise
	int gap;

	printf("New - %d\n", new);


	while (new != 15)
	{
		gap = abs(15-new); //initial gap
		printf("Gap - %d\n", new);		

		drive_goto(dir,-dir); //try 1 direction
		new = pingDistance();

		if (gap < abs(15 - new)) //if did worse
		{
			dir = -dir; //switch direction
		}


		printf("New - %d\n", new);
	}
}

/*void adjust_ir ()
{
	int left, right;
	int gap;
	int dir = 1; //initially try clockwise

	writeIRValues(&left, &right);
	printf("l %d r %d\n",left, right);

	if (left == right)
		return;
	else
	{
		
		printf("Gap init - %d\n", gap);
		drive_goto(dir,-dir);
		writeIRValues(&left, &right);	
		printf("l %d r %d\n",left, right);
		gap = abs(left - right);
		
		while (gap > 0)
		{

		drive_goto(dir,-dir);
		writeIRValues(&left, &right);
		printf("l %d r %d\n",left, right);	
		
		if(gap < abs(left-right))
			dir = -dir; //switch direction

		gap = abs(left - right);
		printf("Gap - %d\n", gap);

		}

	}
}*/

void ping_wall(int i) //check current side for walls
{
	if (pingDistance() < 25) //if closer than 25 - write that there is a wall
	{
		switch (direction)
		{
			case 'n' :
			cells [i].north = 1;
			north_weight = pingDistance(); //save distance from north wall
			break;

			case 'w' :
			cells [i].west = 1;
			west_weight = pingDistance(); //save distance from north wall
			break;

			case 's' :
			cells [i].south = 1;
			south_weight = pingDistance(); //save distance from north wall
			break;

			case 'e' :
			cells [i].east = 1;
			east_weight = pingDistance(); //save distance from north wall
			break;

		}
	}
}

void calibrate_pos();

void check_walls () //check all sides for walls
{
	int i = find_current_cell();

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

	printf("Type - %c \n", cells[i].type);
}

void center_against_walls() //center robot in a cell using two parallel walls
{
	int i = find_current_cell();

	if((cells[i].north && cells[i].south) || (cells[i].east && cells[i].west)) //if there are two parallel walls
	{
		printf("\n");
	}
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
	}

	update_visited();
}

void initialise_cells() //give index and other basic value to cells
{
	int i;

	for (i=0; i < 16; i++)
	{

		if (i<4)
		{
			cells[i].x = 1; 
		}

		else
		{

			if (i<8)
			{
				cells[i].x = 2; 
			}

			else
			{

				if (i<12)
				{
					cells[i].x = 3; 
				}

				else
				{

					cells[i].x = 4; 

				}

			}

		}

	}

	for (i=0; i < 16; i++)
	{
		if (i == 0 || i == 4 || i == 8 || i == 12)
			cells[i].y = 1;

		if (i == 1 || i == 5 || i == 9 || i == 13)
			cells[i].y = 2;

		if (i == 2 || i == 6 || i == 10 || i == 14)
			cells[i].y = 3;

		if (i == 3 || i == 7 || i == 11 || i == 15)
			cells[i].y = 4;

		cells[i].north = 0;
		cells[i].south = 0;
		cells[i].east = 0;
		cells[i].west = 0;
		cells[i].visited = 0;

	}
}

void print_cells() //print data of all cells
{
	int i;

	for (i=0; i < 16; i++)
	{
		printf("x - %d y - %d n,s,e,w: %d%d%d%d visited - %d type - %c\n", cells[i].x, cells[i].y, 
			cells[i].north, cells[i].south,cells[i].east, cells[i].west, cells[i].visited, cells[i].type);
	}
}

void move_out() 
{
	move_forward(); //roll out
	update_visited();
	check_walls();
	determ_type(0);
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

	printf("I came from %c\n", cant);

	if(cells[i].north == 0 && cant != 'n') //find direction in which the robot will move
		will_go = 'n';
	if(cells[i].south == 0 && cant != 's')
		will_go = 's';
	if(cells[i].east == 0 && cant != 'e')
		will_go = 'e';
	if(cells[i].west == 0 && cant != 'w')
		will_go = 'w';


	printf("I will go %c\n", will_go);

	
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
		dest = 'n';
	if (w_av == 1)
		dest = 'w';
	if (s_av == 1)
		dest = 's';
	if (e_av == 1)
		dest = 'e';

	printf("Randomly choose %c\n", dest);

	swap_direction(dest);
}

void rotate_180()
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

void with_least_marks()
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

void cell_analysis()
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

		print_cells();
}

void tremaux()
{
	int i;

	move_forward(); //move out of the initial cell
	check_walls();
	determ_type(0);
	update_visited();


	while(current_x != goal_x || current_y != goal_y)
	{
		cell_analysis();
		move();
	}

	printf("I'm home, master!\n");
}


int main (void)
{

	int left, right;
	int i;

	initialise_cells(); // initiaise cells in the map with indices and false for all walls
	
	tremaux();


	return 0;
}