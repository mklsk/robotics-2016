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

Cell cells[16]; //array of cells

int cell_size_cm = 38;
int cell_size_ticks = DPT * 38;

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

/*
 * Current distances from each wall obtained using the ping sensor
 */
int north_weight;
int south_weight;
int east_weight;
int west_weight;

/*
 * Distances from every wall in the previous cell, mostly used for angle adjustment
 */
int prev_north_weight;
int prev_south_weight;
int prev_east_weight;
int prev_west_weight;

/*
 * Preset for average distance from the wall
 */
int avg_weight = 16;

/*
 * Indicates whether the robot has arrived in the current cell without mapping the previous one, equal to true when the
 * robot traverses a path that it has visited before using Tremaux's algorithm
 */
int checked_walls = 0;

/*
 * Zero radius rotation which calculates the amount of ticks to send to each wheel based on the amount of radians requested
 */
void rotateZeroRadius(double radians) {
    //double radians = PI * (double) angle / 180.0;
    double distancePerWheel = radians * ABW / 2;
    int ticksPerWheel = distancePerWheel / DPT;
    drive_goto(ticksPerWheel, -ticksPerWheel);
    //printf("Ticks rotation: %d\n", ticksPerWheel);
}

/*
 * Function to calculate the new position of the robot and update the previous position
 */
void update_position() //update robots current position
{

    prev_x = current_x;
    prev_y = current_y;

    switch (direction) {
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
}

/*
 * Functions calculating the current cell index in an array based on its X and Y coordinates
 */
int find_cell(int x, int y)
{
    int i;

    for(i = 0; i<16; i++) //find current cell
    {
        if(cells[i].x == x && cells[i].y == y)
            break;
    }

    return i;
}

int find_current_cell() { return find_cell(current_x, current_y); }

int find_prev_cell() { return find_cell(prev_x, prev_y); }

/*
 * Update number of times the current cell was visited
 */
void update_visited() {
    int i = find_current_cell();
    cells[i].visited++;
}

/*
 * Measure distance from a wall in current direction
 */
int pingDistance() {
    return ping_cm(8);;
}

/*
 * Move one cell forward in current direction
 */
void move_forward() {
    drive_goto(cell_size_ticks, cell_size_ticks);
}

/*
 * Turn clockwise or anti-clockwise
 */
void turn(char dir) {
    int left, right;
    switch (dir) {
        case 'a' :
            drive_goto(-26, 25);

            switch (direction) {
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
            drive_goto(25, -26);
            switch (direction) {
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

/*
 * Update the Cell struct representing the wall the robot is currently facing using the ping sensor. Additionally,
 * notify te neighbour about the absence of the wall
 */
void ping_wall(int i)
{
    int temp_ping_dist;
    switch (direction) {
        case 'n' :
            temp_ping_dist = pingDistance();
            if (temp_ping_dist < 30) {
                cells[i].north = 1;
                //printf("wall on n\n");
            } else {
                cells[i].north = 0;
                int neighbour = find_cell(cells[i].x, cells[i].y + 1);
                if (neighbour != -1) cells[neighbour].south = 0;
            }
            north_weight = temp_ping_dist; //save distance from north wall
            break;

        case 'w' :
            temp_ping_dist = pingDistance();
            if (temp_ping_dist < 30) {
                cells[i].west = 1;
                //printf("wall on w\n");
            } else {
                cells[i].west = 0;
                int neighbour = find_cell(cells[i].x - 1, cells[i].y);
                if (neighbour != -1) cells[neighbour].east = 0;
            }
            west_weight = temp_ping_dist; //save distance from north wall
            break;

        case 's' :
            temp_ping_dist = pingDistance();
            if (temp_ping_dist < 30) {
                cells[i].south = 1;
                //printf("wall on s\n");
            } else {
                cells[i].south = 0;
                int neighbour = find_cell(cells[i].x, cells[i].y - 1);
                if (neighbour != -1) cells[neighbour].north = 0;
            }
            south_weight = temp_ping_dist; //save distance from north wall
            break;

        case 'e' :
            temp_ping_dist = pingDistance();
            if (temp_ping_dist < 30) {
                cells[i].east = 1;
                //printf("wall on e\n");
            } else {
                cells[i].east = 0;
                int neighbour = find_cell(cells[i].x + 1, cells[i].y);
                if (neighbour != -1) cells[neighbour].west = 0;
            }
            east_weight = temp_ping_dist; //save distance from north wall
            break;

    }
}

/*
 * Inverse cosine approximation using Lagrange polynomial, mostly used for angle calculations
 */
double acos(double x) {
    return (-0.69813170079773212 * x * x - 0.87266462599716477) * x + 1.5707963267948966;
}

/*
 * Angle adjustment using simple geometry, used to fix robots positions against the walls
 */
void adjust_angle_triangle(int x1, int x2) {
    if (prev_x == current_x && prev_y == current_y) return;
    int temp_diff = x1 - x2;
    double cosine = (double) temp_diff / (double) cell_size_cm;
    double radians = PI / 2.0 - acos(cosine);
    rotateZeroRadius(radians);
}

/*
 * Turn robot to a desired direction
 */
void swap_direction(char goal)
{
    if (goal != direction) {

        switch (goal) {
            case ('n'):
                switch (direction) {
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
                switch (direction) {
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
                switch (direction) {
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
                switch (direction) {
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

/*
 * Adjust the distance to default distance using the wall the robot is currently pointing to
 */
void adjust_one_wall(int *temp_weight) {
    ping_wall(find_current_cell());
    int adj_weight = avg_weight;
    adj_weight = 16;
    int difference = *temp_weight - adj_weight;
    int ticks = difference * 10.0 / 3.25;
    drive_goto(ticks, ticks);
    *temp_weight = avg_weight;
}

/*
 * Measure distances form walls in a current cell.
 * Values of distances from the walls are saved in global variables
 */
void check_wall_weights() {

    int i = find_current_cell();

    prev_north_weight = north_weight;
    prev_east_weight = east_weight;
    prev_south_weight = south_weight;
    prev_west_weight = west_weight;

    switch (direction) {
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
    char memory_direction = direction;
    if (cells[i].north != 0) {
        swap_direction('n');
        ping_wall(i);
    }
    if (cells[i].east != 0) {
        swap_direction('e');
        ping_wall(i);
    }
    if (cells[i].south != 0) {
        swap_direction('s');
        ping_wall(i);
    }
    if (cells[i].west != 0) {
        swap_direction('w');
        ping_wall(i);
    }
    swap_direction(memory_direction);

}

/*
 * Check each side of the cell for having walls
 * Adjust robots position in a cell using one or two walls
 */
void check_walls()
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
    if (cells[i].north) {
        adj_dir_1 = 'n';
        adj_weight_1 = &north_weight;
    } else if (cells[i].south) {
        adj_dir_1 = 's';
        adj_weight_1 = &south_weight;
    }
    if (cells[i].east) {
        adj_dir_2 = 'e';
        adj_weight_2 = &east_weight;
    } else if (cells[i].west) {
        adj_dir_2 = 'w';
        adj_weight_2 = &west_weight;
    }


    int adj_weight_prev;
    int adj_weight_next;
    int do_adj = 0;
    if (cells[i].north == 1 && cells[z].north == 1) {
        adj_weight_next = north_weight;
        adj_weight_prev = prev_north_weight;
        do_adj = 1;
    } else if (cells[i].east == 1 && cells[z].east == 1) {
        adj_weight_next = -east_weight;
        adj_weight_prev = -prev_east_weight;
        do_adj = 1;
    } else if (cells[i].south == 1 && cells[z].south == 1) {
        adj_weight_next = -south_weight;
        adj_weight_prev = -prev_south_weight;
        do_adj = 1;
    } else if (cells[i].west == 1 && cells[z].west == 1) {
        adj_weight_next = west_weight;
        adj_weight_prev = prev_west_weight;
        do_adj = 1;
    }

    if (do_adj && checked_walls) adjust_angle_triangle(adj_weight_prev, adj_weight_next);


    switch (direction) {
        case 's':
            y_diff = -1 * y_diff;
            break;
        case 'w':
            x_diff = -1 * x_diff;
            break;
    }


    double adjustment = 0;

    if (abs(y_diff) < diff_treshold && y_sum < sum_treshold) {
        if (adj_dir_2 != '\0') {
            swap_direction(adj_dir_2);
            adjust_one_wall(adj_weight_2);
        }
        if (direction != 'n' && direction != 's') swap_direction('n');
        adjustment = (double) y_diff / 2.0;
        north_weight = y_sum / 2;
        south_weight = y_sum / 2;
        if (avg_weight == -1) avg_weight = y_sum / 2;
        else {
            avg_weight += y_sum / 2;
            avg_weight /= 2;
        }
    } else if (abs(x_diff) < diff_treshold && x_sum < sum_treshold) {
        if (adj_dir_1 != '\0') {
            swap_direction(adj_dir_1);
            adjust_one_wall(adj_weight_1);
        }
        if (direction != 'e' && direction != 'w') swap_direction('e');
        adjustment = (double) x_diff / 2.0;
        west_weight = x_sum / 2;
        east_weight = x_sum / 2;
        if (avg_weight == -1) avg_weight = x_sum / 2;
        else {
            avg_weight += x_sum / 2;
            avg_weight /= 2;
        }
    } else if (avg_weight != -1) {
        if (adj_dir_1 != '\0') {
            swap_direction(adj_dir_1);
            adjust_one_wall(adj_weight_1);
        }
        if (adj_dir_2 != '\0') {
            swap_direction(adj_dir_2);
            adjust_one_wall(adj_weight_2);
        }
    }

    int ticks = adjustment * 10.0 / 3.25;
    drive_goto(ticks, ticks);
    swap_direction(memory_direction);

}

/*
 * Determine if cell is an element of path (2 walls), a dead end (3 walls) or junction (1 or 0)
 */
void determ_type(int i)
{
    int sum = cells[i].north + cells[i].south + cells[i].east + cells[i].west;
    if (sum == 3)
        cells[i].type = 'd';
    if (sum == 2)
        cells[i].type = 'p';
    if (sum < 2)
        cells[i].type = 'j';
}

/*
 * Move robot forward, update current position and perform adjustments if the cell has been visited before
 */
void move()
{
    move_forward();
    update_position();
    int i = find_current_cell();
    if (cells[i].visited == 0) {
        check_walls();
        determ_type(i);
        checked_walls = 1;
    } else {
        checked_walls = 0;
        int i = find_current_cell();
        char adj_dir_1 = '\0';
        int *adj_weight_1;
        char adj_dir_2 = '\0';
        int *adj_weight_2;
        if (cells[i].north) {
            adj_dir_1 = 'n';
            adj_weight_1 = &north_weight;
        } else if (cells[i].south) {
            adj_dir_1 = 's';
            adj_weight_1 = &south_weight;
        }
        if (cells[i].east) {
            adj_dir_2 = 'e';
            adj_weight_2 = &east_weight;
        } else if (cells[i].west) {
            adj_dir_2 = 'w';
            adj_weight_2 = &west_weight;
        }
        char mem_dir = direction;
        if (adj_dir_1 != '\0') {
            swap_direction(adj_dir_1);
            adjust_one_wall(adj_weight_1);
        }
        if (adj_dir_2 != '\0') {
            swap_direction(adj_dir_2);
            adjust_one_wall(adj_weight_2);
        }
        swap_direction(mem_dir);
    }


    update_visited();
}

/*
 * Assign index and other basic value to cell structures
 */
void initialise_cells()
{
    int i;

    for (i = 0; i < 16; i++) {
        cells[i].x = i / 4 + 1;
        cells[i].y = i % 4 + 1;

        cells[i].north = -1;
        cells[i].south = -1;
        cells[i].east = -1;
        cells[i].west = -1;

        cells[i].visited = 0;
    }
}

/*
 * Find a direction from which robot came, and thus shouldn't go there
 */
char cant_go()
{
    char cant_go = '\0';
    if (direction == 'n')
        cant_go = 's';
    if (direction == 's')
        cant_go = 'n';
    if (direction == 'e')
        cant_go = 'w';
    if (direction == 'w')
        cant_go = 'e';
    return cant_go;
}

/*
 * Choose direction of new movement, if a robot is on an element of path
 * After it is chosen, turn towards the direction
 */
void choose_direction_p()
{
    char cant = cant_go();
    char will_go;
    int i = find_current_cell();

    if (cells[i].north == 0 && cant != 'n')
        will_go = 'n';
    if (cells[i].south == 0 && cant != 's')
        will_go = 's';
    if (cells[i].east == 0 && cant != 'e')
        will_go = 'e';
    if (cells[i].west == 0 && cant != 'w')
        will_go = 'w';
    swap_direction(will_go);
}

/*
 * Choose a random direction in case if robot faces a new junction
 */
void choose_direct_rand_j()
{
    char cant = cant_go();
    char dest;
    int i = find_current_cell();
    int n_av = 1;
    int s_av = 1;
    int e_av = 1;
    int w_av = 1;

    switch (cant) {
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
    if (n_av == 1) {
        dest = 'n';
    }
    if (w_av == 1) {
        dest = 'w';
    }
    if (s_av == 1) {
        dest = 's';
    }
    if (e_av == 1) {
        dest = 'e';
    }
    swap_direction(dest);
}

/*
 * Rotate the robot 180 degrees
 */
void rotate_180()
{
    switch (direction) {
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

/*
 * Find a neighbour of a current cell that was visited the least number of times
 * Turns the robot towards it
 */
void with_least_marks()
{
    int min = 3;
    char min_dir;
    int i = find_current_cell();
    int j;


    if (cells[i].east == 0) //if can enter cell on east
    {
        current_x++; //find cell on east
        j = find_current_cell();
        current_x--;

        if (cells[j].visited < min) {
            min = cells[j].visited;
            min_dir = 'e';
        }
    }

    if (cells[i].north == 0) //if can enter cell on north
    {
        current_y++; //find cell on north
        j = find_current_cell();
        current_y--;

        if (cells[j].visited < min) {
            min = cells[j].visited;
            min_dir = 'n';
        }
    }

    if (cells[i].west == 0) //if can enter cell on west
    {
        current_x--; //find cell on west
        j = find_current_cell();
        current_x++;

        if (cells[j].visited < min) {
            min = cells[j].visited;
            min_dir = 'w';
        }
    }

    if (cells[i].south == 0) //if can enter cell on south
    {
        current_y--; //find cell on south
        j = find_current_cell();
        current_y++;

        if (cells[j].visited < min) {
            min = cells[j].visited;
            min_dir = 's';
        }
    }


    swap_direction(min_dir);
}

/*
 * Behaviour of cells for Tremaux algorithm
 */
void cell_analysis() //determine type of cell
{
    int i = find_current_cell();

    if (cells[i].type == 'p') //if path  - just follow it
    {
        choose_direction_p();
    }

    if (cells[i].type == 'd') // if dead end - turn 180
    {
        rotate_180();
        update_visited();

    }

    if (cells[i].type == 'j') //if junction
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


}

/*
 * Perform Tremaux algorithm until all 3 goal cells are visited
 *
 * Determine behavior of the robot in a cell according to its type
 * (path/dead end/junction)
 *
 * if path - go to the only possible direction
 * if dead end - rotate 180
 * if coming from a visited path - go to neighbour with least marks on a junction
 * if coming from a new path
 *  if junction is new - randomly choose direction
 *  if not - rotate 180
 */
void tremaux() //do tremaux algorithm
{


    move_forward(); //move out of the initial cell
    check_walls();
    determ_type(0);
    update_visited();

    int k, m, l;

    k = find_cell(goal1_x, goal1_y); //get goal cells
    m = find_cell(goal2_x, goal2_y);
    l = find_cell(goal3_x, goal3_y);


    while (cells[k].visited == 0 || cells[l].visited == 0 || cells[m].visited == 0) {
        cell_analysis();

        move();

    }
}

/*
 * Initialises cost matrix with values of walls from the array of cells
 */
void create_matrices()
{
    int i, j;
    int n, s, e, w;
    int n_can, s_can, e_can, w_can;
    int x, y;

    for (i = 0; i < 16; i++) {
        for (j = 0; j < 16; j++) {
            if (i == j) {
                cost_matrix[i][j] = 0; //no cost of going to itself
            }

            else {
                cost_matrix[i][j] = 99; //by default - no connection for all nodes
            }

            pass_matrix[i][j] = -1;
        }
    }

    for (i = 0; i < 16; i++) {
        n_can = 1;
        s_can = 1;
        e_can = 1;
        w_can = 1;



        if (cells[i].y < 4) //check if cell on the north is possible
        {
            x = cells[i].x;
            y = cells[i].y + 1;
            n = find_cell(x, y); //get cell on the north

        }

        if (cells[i].y > 1) //check if cell on the south is possible
        {
            x = cells[i].x;
            y = cells[i].y - 1;
            s = find_cell(x, y); //get cell on the south

        }

        if (cells[i].x < 4) //check if cell on the east is possible
        {
            x = cells[i].x + 1;
            y = cells[i].y;
            e = find_cell(x, y); //get cell on the east

        }

        if (cells[i].x > 1) //check if cell on the south is possible
        {
            x = cells[i].x - 1;
            y = cells[i].y;
            w = find_cell(x, y); //get cell on the south


        }


        if (cells[i].north == 0 && n_can == 1) //if no wall and cell was visited, write to table
        {
            cost_matrix[i][n] = 1;
            cost_matrix[n][i] = 1;
        }

        if (cells[i].south == 0 && s_can == 1) //if no wall and cell was visited, write to table
        {
            cost_matrix[i][s] = 1;
            cost_matrix[s][i] = 1;
        }

        if (cells[i].east == 0 && e_can == 1) //if no wall and cell was visited, write to table
        {
            cost_matrix[i][e] = 1;
            cost_matrix[e][i] = 1;
        }

        if (cells[i].west == 0 && w_can == 1) //if no wall and cell was visited, write to table
        {
            cost_matrix[i][w] = 1;
            cost_matrix[w][i] = 1;
        }
    }
}

/*
 * Give initial values of -1 to all elements of array that will hold the shortest path
 */
void path_init() {
    int i;
    for (i = 0; i < 16; i++) {
        short_path[i] = -1;
    }
}

/*
 * Function responsible for traversing of the shortest path generated by the Dijkstra's algorithm
 */
void follow_shortest(int do_adjustment) {


    int i;
    for (i = 0; i < 16; i++) {
    }


    int k = 0;
    while (short_path[k] != -1) {
        i = find_current_cell();
        int z = find_prev_cell();
        Cell cell = cells[short_path[k]];

        if (cell.x == current_x && cell.y == current_y) {
            k++;
            continue;
        }
        if (do_adjustment) {

            check_wall_weights();

            int adj_weight_prev;
            int adj_weight_next;
            int do_adj = 0;
            if (cells[i].north == 1 && cells[z].north == 1) {
                adj_weight_next = north_weight;
                adj_weight_prev = prev_north_weight;
                do_adj = 1;
            } else if (cells[i].east == 1 && cells[z].east == 1) {
                adj_weight_next = east_weight;
                adj_weight_prev = prev_east_weight;
                do_adj = 1;
            } else if (cells[i].south == 1 && cells[z].south == 1) {
                adj_weight_next = south_weight;
                adj_weight_prev = prev_south_weight;
                do_adj = 1;
            } else if (cells[i].west == 1 && cells[z].west == 1) {
                adj_weight_next = west_weight;
                adj_weight_prev = prev_west_weight;
                do_adj = 1;
            }

            if (do_adj) adjust_angle_triangle(adj_weight_prev, adj_weight_next);


            // Pick adjustment wall
            char adj_dir_1 = '\0';
            int *adj_weight_1;
            char adj_dir_2 = '\0';
            int *adj_weight_2;
            if (cells[i].north) {
                adj_dir_1 = 'n';
                adj_weight_1 = &north_weight;
            } else if (cells[i].south) {
                adj_dir_1 = 's';
                adj_weight_1 = &south_weight;
            }
            if (cells[i].east) {
                adj_dir_2 = 'e';
                adj_weight_2 = &east_weight;
            } else if (cells[i].west) {
                adj_dir_2 = 'w';
                adj_weight_2 = &west_weight;
            }

            char mem_dir = direction;
            if (adj_dir_1 != '\0') {
                swap_direction(adj_dir_1);
                adjust_one_wall(adj_weight_1);
            }
            if (adj_dir_2 != '\0') {
                swap_direction(adj_dir_2);
                adjust_one_wall(adj_weight_2);
            }
            swap_direction(mem_dir);
        }

        prev_x = current_x;
        prev_y = current_y;


        int j = 1;

        if (cell.x != current_x) {
            if (cell.x > current_x) {
                swap_direction('e');
                if (!do_adjustment) {
                    while (short_path[k + j] != -1) {
                        int cell_curr_temp = short_path[k + j - 1];
                        int cell_next_temp = short_path[k + j];
                        if (cells[cell_next_temp].x > cells[cell_curr_temp].x) {
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
                if (!do_adjustment) {
                    while (short_path[k + j] != -1) {
                        int cell_curr_temp = short_path[k + j - 1];
                        int cell_next_temp = short_path[k + j];
                        if (cells[cell_next_temp].x < cells[cell_curr_temp].x) {
                            j++;
                        } else {
                            break;
                        }
                    }
                }
                drive_goto(cell_size_ticks * j, cell_size_ticks * j);
                current_x -= j;
            }
        } else if (cell.y != current_y) {
            if (cell.y > current_y) {
                swap_direction('n');
                if (!do_adjustment) {
                    while (short_path[k + j] != -1) {
                        int cell_curr_temp = short_path[k + j - 1];
                        int cell_next_temp = short_path[k + j];
                        if (cells[cell_next_temp].y > cells[cell_curr_temp].y) {
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
                if (!do_adjustment) {
                    while (short_path[k + j] != -1) {
                        int cell_curr_temp = short_path[k + j - 1];
                        int cell_next_temp = short_path[k + j];
                        if (cells[cell_next_temp].y < cells[cell_curr_temp].y) {
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

/*
 * Update the map in the memory by replacing all unknowns with walls to improve the performance of Dijkstra's algorithm
 */
void convert_unknown_to_walls() {
    int i;
    for (i = 0; i < 16; i++) {
        if (cells[i].north == -1) cells[i].north = 1;
        if (cells[i].south == -1) cells[i].south = 1;
        if (cells[i].east == -1) cells[i].east = 1;
        if (cells[i].west == -1) cells[i].west = 1;
    }
}

/*
 * Run Dijkstra's algorithm that adds indices of cells in a shortest path to short_path array
 * Plots a path from startnode to endnode
 */
void dijkstra(int G[MAX][MAX], int n, int startnode, int endnode) {
    int cost[MAX][MAX], distance[MAX], pred[MAX];
    int visited[MAX], count, mindistance, nextnode, i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            if (G[i][j] == 0)
                cost[i][j] = INFINITY;
            else
                cost[i][j] = G[i][j];

    for (i = 0; i < n; i++) {
        distance[i] = cost[startnode][i];
        pred[i] = startnode;
        visited[i] = 0;
    }
    distance[startnode] = 0;
    visited[startnode] = 1;
    count = 1;
    while (count < n - 1) {
        mindistance = INFINITY;
        for (i = 0; i < n; i++)
            if (distance[i] < mindistance && !visited[i]) {
                mindistance = distance[i];
                nextnode = i;
            }
        visited[nextnode] = 1;
        for (i = 0; i < n; i++)
            if (!visited[i]) if (mindistance + cost[nextnode][i] < distance[i]) {
                distance[i] = mindistance + cost[nextnode][i];
                pred[i] = nextnode;
            }
        count++;
    }

    for (i = 0; i < n; i++)
        if (i == endnode) {
            int count = 1;
            int temp[16];
            temp[0] = i;
            j = i;

            do {
                j = pred[j];
                temp[count] = j;
                count++;
            }
            while (j != startnode);

            int x = 0;


            x = 0;
            int y;
            count--;


            for (y = count; y >= 0; y--) {
                short_path[x] = temp[count];
                count--;
                x++;
            }


        }
}

int main(void) {


    initialise_cells(); // initiaise cells in the map with indices and false for all walls

    tremaux();

    convert_unknown_to_walls();

    create_matrices();

    path_init();

    dijkstra(cost_matrix, 16, find_current_cell(), 0);
    follow_shortest(1);

    int i = find_current_cell();

    swap_direction('s');
    move_forward();

    swap_direction('n');

    for (i = 0; i < 3; i++) {
        high(26);
        pause(500);
        low(26);
        pause(500);
    }

    move_forward();
    path_init();
    dijkstra(cost_matrix, 16, 0, 15);
    follow_shortest(0);

    for (i = 0; i < 3; i++) {
        high(26);
        pause(500);
        low(26);
        pause(500);
    }

    return 0;
}