#include <GL\glew.h>
#include <GL\freeglut.h>

#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

using namespace std;
const int screenW = 800, screenH = 800;
const int tileSize = 10;
const int gridW = screenW / tileSize, gridH = screenH / tileSize;

double myTime = 0;

bool canSolve = false;

void setHSV(double h, double s, double v) {
	double hh, p, q, t, ff;
	long i;
	double r, g, b;

	if (s <= 0.0) {
		r = v;
		g = v;
		b = v;
	}
	hh = h;
	//if (hh >= 360.0) hh = 0.0;
	while (hh >= 360.0) hh -= 360;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = v * (1.0 - s);
	q = v * (1.0 - (s * ff));
	t = v * (1.0 - (s * (1.0 - ff)));

	switch (i) {
	case 0:
		r = v;
		g = t;
		b = p;
		break;
	case 1:
		r = q;
		g = v;
		b = p;
		break;
	case 2:
		r = p;
		g = v;
		b = t;
		break;
	case 3:
		r = p;
		g = q;
		b = v;
		break;
	case 4:
		r = t;
		g = p;
		b = v;
		break;
	case 5:
	default:
		r = v;
		g = p;
		b = q;
		break;
	}
	glColor3d(r, g, b);
}
uint8_t col = 1;
typedef struct GridTile {
	uint8_t x, y;
	uint16_t gc = 0, hc, fc = 0;
	bool wall = false, open = false, closed = false;
	GridTile* parent = NULL;

	bool solution = false;

	GridTile* adjNeighbors[4];
	GridTile* diagNeighbors[4];

	GridTile() {}
	GridTile(uint8_t x_, uint8_t y_) {
		x = x_;
		y = y_;
	}

	void draw(bool isStart, bool isTarget, uint8_t ts) {
		if (isStart) glColor3f(0.5f, 0.7f, 0.5f);
		else if (isTarget) glColor3f(0.7f, 0.5f, 0.5f);
		else if (solution) glColor3f(0.1f, 0.7f, 0.5f);
		else if (wall) glColor3f(0.3f, 0.3f, 0.3f);
		else if (open) glColor3f(0.1f, 0.1f, 0.5f);
		else if (closed && col == 0) setHSV(gc / 10.0, 1.0, 1.0);
		else if (closed && col == 1) glColor3f(0.4f, 0.1f, 0.4f);
		else glColor3f(0.5f, 0.5f, 0.6f);
		//CHANGE FOR SQUARE BORDER
		//glRectd(x * ts + (ts * 0.05), y * ts + (ts * 0.05), x * ts + (ts * 0.95), y * ts + (ts * 0.95));
		glRectd(x * ts, y * ts, x * ts + ts, y * ts + ts);
	}
	friend std::ostream& operator<< (std::ostream& out, const GridTile* gt) {
		int count = 0;
		for (int i = 0; i < 4; i++) {
			if (gt->adjNeighbors[i] != NULL) count++;
		}
		for (int i = 0; i < 4; i++) {
			if (gt->diagNeighbors[i] != NULL) count++;
		}
		cout << "Tile: " << &gt << " at position: " << (int)gt->x << ", " << (int)gt->y << endl;
		cout << "GC: " << gt->gc << " HC: " << gt->hc << " FC: " << gt->fc << endl;
		cout << ((gt->open) ? "Open" : "Not Open") << endl << ((gt->closed) ? "Closed" : "Not Closed") << endl;
		cout << "Amount of neighbors: " << count << endl;
		if (gt->parent != NULL) cout << "Parent Coords: " << (int)gt->parent->x << ", " << (int)gt->parent->y << endl;
		cout << endl;
		return out;
	}
} tile;

tile all[gridW][gridH];
vector<tile*> open;
tile* start;
tile* current;
tile* target;

//void initialize();
//void keyboard(unsigned char key, int x, int y);
//void mouse(int button, int state, int x, int y);
//void drag(int x, int y);
//void mouseWheel(int wheel, int direction, int x, int y);
//void resize(int w_, int h_);
//void draw();
//void update();

void initialize() {
	for (int j = 0; j < gridH; j++) {
		for (int i = 0; i < gridW; i++) {
			all[i][j].x = i;
			all[i][j].y = j;
		}
	}
	cout << ((canSolve) ? "Solving" : "Stand By") << endl;

	glOrtho(0, screenW - 1, 0, screenH - 1, -1, 1);
}
uint8_t min(uint8_t a, uint8_t b) {
	return ((a < b) ? a : b);
}
void initHCostAndNeighbors() {
	//FOR CALCULATING DIST TO GOAL
	//h = min(dx, dy) * 14 + abs(dx - dy) * 10

	//where
	//val dx = abs(goalXcoord - nodeXcoordinate)
	//val dy = abs(goalYcoord - nodeYcoordinate)
	for (int j = 0; j < gridH; j++) {
		for (int i = 0; i < gridW; i++) {
			uint8_t dx = abs(target->x - all[i][j].x);
			uint8_t dy = abs(target->y - all[i][j].y);
			all[i][j].hc = min(dx, dy) * 14 + abs(dx - dy) * 10;

			//ASSIGN NEIGHBORS
			//Adjacent
			all[i][j].adjNeighbors[0] = (i > 0) ? ((!all[i - 1][j].wall) ? &all[i - 1][j] : NULL) : NULL; //LEFT
			all[i][j].adjNeighbors[1] = (i < gridW - 1) ? ((!all[i + 1][j].wall) ? &all[i + 1][j] : NULL) : NULL; //RIGHT
			all[i][j].adjNeighbors[2] = (j > 0) ? ((!all[i][j - 1].wall) ? &all[i][j - 1] : NULL) : NULL; //BOTTOM
			all[i][j].adjNeighbors[3] = (j < gridH - 1) ? ((!all[i][j + 1].wall) ? &all[i][j + 1] : NULL) : NULL; //TOP

			//Diagnol
			all[i][j].diagNeighbors[0] = (i > 0 && j > 0) ? ((!all[i - 1][j - 1].wall) ? &all[i - 1][j - 1] : NULL) : NULL; //BOTLEFT
			all[i][j].diagNeighbors[1] = (i < gridW - 1 && j > 0) ? ((!all[i + 1][j - 1].wall) ? &all[i + 1][j - 1] : NULL) : NULL; //BOTRIGHT
			all[i][j].diagNeighbors[2] = (i > 0 && j < gridH - 1) ? ((!all[i - 1][j + 1].wall) ? &all[i - 1][j + 1] : NULL) : NULL; //TOPLEFT
			all[i][j].diagNeighbors[3] = (i < gridW - 1 && j < gridH - 1) ? ((!all[i + 1][j + 1].wall) ? &all[i + 1][j + 1] : NULL) : NULL; //TOPRIGHT
		}
	}
	current = start;
}
void drawFinishPath(tile* t) {
	if (t->parent == NULL) return;
	else {
		if (t != target) t->solution = true;
		return drawFinishPath(t->parent);
	}
}
void update() {
	//Make current tile the lowest fcost in open
	//Add other tiles with equal fcost to lowest
	vector<tile*> lowest;
	if (open.size() > 0) lowest.push_back(open[0]);
	else {
		canSolve = false;
		cout << "NO POSSIBLE PATH" << endl;
		return;
	}
	for (int i = 1; i < open.size(); i++) {
		if (open[i]->fc == lowest[0]->fc)
			lowest.push_back(open[i]);
		else if (open[i]->fc < lowest[0]->fc) {
			lowest.clear();
			lowest.push_back(open[i]);
		}
	}
	if (lowest.size() < 1) {
		lowest.clear();
		return;
	}
	//Use tiles in lowest to compare hcosts
	current = lowest[0];
	if (lowest.size() > 1) {
		for (int i = 1; i < lowest.size(); i++) {
			if (lowest[i]->hc < current->hc)
				current = lowest[i];
		}
	}
	lowest.clear();
	//Erase current from open
	for (int i = 0; i < open.size(); i++) {
		if (open[i] == current) {
			open.erase(open.begin() + i);
			break;
		}
	}
	current->open = false;
	current->closed = true;
	//Test current with target
	if (current == target) {
		canSolve = false;
		cout << "PATH FOUND" << endl;
		drawFinishPath(current);
		return;
	}
	else {
		for (tile* n : current->adjNeighbors) {
			if (n == NULL || n->wall || n->closed) continue;
			if ((current->gc + 10 < n->gc && n->gc != 0) || !n->open) {
				n->gc = current->gc + 10;
				n->fc = n->gc + n->hc;
				n->parent = current;
				if (!n->open) {
					n->open = true;
					open.push_back(n);
				}
			}
		}
		for (tile* n : current->diagNeighbors) {
			if (n == NULL || n->wall || n->closed) continue;
			if ((current->gc + 14 < n->gc && n->gc != 0) || !n->open) {
				n->gc = current->gc + 14;
				n->fc = n->gc + n->hc;
				n->parent = current;
				if (!n->open) {
					n->open = true;
					open.push_back(n);
				}
			}
		}
	}
}
void draw() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//DRAW
	for (int j = 0; j < gridH; j++) {
		for (int i = 0; i < gridW; i++) {
			all[i][j].draw((&all[i][j] == start) ? true : false, (&all[i][j] == target) ? true : false, tileSize);
		}
	}
	//UPDATE
	if (canSolve)
		update();

	glFlush();
	glutSwapBuffers();
}
void reset() {
	for (int j = 0; j < gridH; j++) {
		for (int i = 0; i < gridW; i++) {
			all[i][j].gc = 0;
			all[i][j].hc = 0;
			all[i][j].fc = 0;

			//all[i][j].wall = false;
			all[i][j].open = false;
			all[i][j].closed = false;

			all[i][j].solution = false;

			all[i][j].parent = NULL;
		}
	}
}

int8_t flags = 0b0000'0001;
const int8_t aStart = 0b0000'0001;
const int8_t aTarget = 0b0000'0010;
const int8_t aWalls = 0b0000'0100;
int windowID;
void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'q':
		flags |= aStart;
		flags &= ~aTarget;
		flags &= ~aWalls;
		break;
	case 'w':
		flags &= ~aStart;
		flags |= aTarget;
		flags &= ~aWalls;
		break;
	case 'e':
		flags &= ~aStart;
		flags &= ~aTarget;
		flags |= aWalls;
		break;
		//SWITCH COLOR MODE
	case 'a':
		if (col == 0) col = 1;
		else col = 0;
		break;
		//START SOLVING
	case 's':
		if (!canSolve) {
			if (start && target) {
				initHCostAndNeighbors();
				canSolve = true;
			}
			else {
				cout << "Need start and target to solve" << endl;
				break;
			}
			cout << ((canSolve) ? "Solving" : "Stand By") << endl;
		}
		break;
		//RESET KEY
	case 'r':
		canSolve = false;
		cout << ((canSolve) ? "Solving" : "Stand By") << endl;
		open.clear();
		open.push_back(start);
		current = NULL;
		reset();
		break;
		//CLEAR WALLS
	case 'c':
		if (!canSolve) {
			for (int j = 0; j < gridH; j++) {
				for (int i = 0; i < gridW; i++)
					all[i][j].wall = false;
			}
		}
		break;
		//TEST UPDATE
	case 'u':
		if (canSolve) update();
		else cout << "Need start and target to solve" << endl;
		break;
	case 'x':
		glutDestroyWindow(windowID);
		exit(0);
		break;
	}
}

bool leftDown = false, rightDown = false;
void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN && !canSolve) {
		if ((flags & aStart) && !all[x / tileSize][(screenH - y) / tileSize].wall && &all[x / tileSize][(screenH - y) / tileSize] != target) {
			if (start) start->open = false;
			open.clear();
			start = &all[x / tileSize][(screenH - y) / tileSize];
			start->open = true;
			open.push_back(start);
		}
		else if ((flags & aTarget) && !all[x / tileSize][(screenH - y) / tileSize].wall && &all[x / tileSize][(screenH - y) / tileSize] != start) {
			target = &all[x / tileSize][(screenH - y) / tileSize];
		}
		else if ((flags & aWalls) && &all[x / tileSize][(screenH - y) / tileSize] != start && &all[x / tileSize][(screenH - y) / tileSize] != target) {
			all[x / tileSize][(screenH - y) / tileSize].wall = true;
			int a = x / tileSize, b = (screenH - y) / tileSize;
			if (a + 1 < gridW && &all[a][b] != start && &all[a][b] != target) all[a + 1][b].wall = true;
			if (b + 1 < gridH && &all[a][b] != start && &all[a][b] != target) all[a][b + 1].wall = true;
			if (a + 1 < gridW && b + 1 < gridH && &all[a][b] != start && &all[a][b] != target) all[a + 1][b + 1].wall = true;
		}
		leftDown = true;
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN && !canSolve) {
		if ((flags & aStart) && start == &all[x / tileSize][(screenH - y) / tileSize]) {
			if (start) start->open = false;
			open.clear();
			start = NULL;
		}
		else if ((flags & aTarget) && target == &all[x / tileSize][(screenH - y) / tileSize]) {
			target = NULL;
		}
		else if (flags & aWalls) all[x / tileSize][(screenH - y) / tileSize].wall = false;
		rightDown = true;
	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP && !canSolve)
		leftDown = false;
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP && !canSolve)
		rightDown = false;
}
void drag(int x, int y) {
	if ((flags & aWalls) && leftDown && &all[x / tileSize][(screenH - y) / tileSize] != start && &all[x / tileSize][(screenH - y) / tileSize] != target) {
		all[x / tileSize][(screenH - y) / tileSize].wall = true;
		int a = x / tileSize, b = (screenH - y) / tileSize;
		if (a + 1 < gridW && &all[a][b] != start && &all[a][b] != target) all[a + 1][b].wall = true;
		if (b + 1 < gridH && &all[a][b] != start && &all[a][b] != target) all[a][b + 1].wall = true;
		if (a + 1 < gridW && b + 1 < gridH && &all[a][b] != start && &all[a][b] != target) all[a + 1][b + 1].wall = true;
	}
	else if ((flags & aWalls) && rightDown) all[x / tileSize][(screenH - y) / tileSize].wall = false;
}
void mouseWheel(int wheel, int direction, int x, int y) {

}
void resize(int w_, int h_) {
	glViewport(0, 0, w_, h_);
	//w = w_;
	//h = h_;
}
void idle() {
	myTime += 0.01;
	if (myTime > 10)
		myTime = 0.0;
	glutPostRedisplay();
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	glutInitWindowSize(screenW, screenH);
	//glutFullScreen();
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - screenW) / 2.0, (glutGet(GLUT_SCREEN_HEIGHT) - screenH) / 2.0 - 50);
	windowID = glutCreateWindow("First Window");

	initialize();
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(drag);
	glutMouseWheelFunc(mouseWheel);
	glutReshapeFunc(resize);
	glutDisplayFunc(draw);
	glutIdleFunc(idle);

	glutMainLoop();
	return 0;
}