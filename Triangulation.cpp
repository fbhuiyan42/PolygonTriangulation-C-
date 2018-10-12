#include <iostream>
#include<fstream>
#include <string.h>
#include <algorithm>
#include <list>
#include <set>
#include<GL/glut.h>

using namespace std;
ifstream fin ("input.txt");

#define Left 1
#define Right -1
#define START 1
#define END 2
#define SPLIT 3
#define MERGE 4
#define REGULAR 0
#define BLACK 0, 0, 0


class Point
{
    public:
        float x;
        float y;
};

class Vertex
{
    public:
        Point point;
        int previous;
        int next;
};

class Polygon
{
    public:

        Point *points;
        int polygonSize;

        Polygon()
        {
            points = NULL;
            polygonSize = 0;
        }

        Point &GetPoint(long i)
        {
            return points[i];
        }

        Point& operator[] (int i)
        {
            return points[i];
        }

        void Triangle(Point &point1, Point &point2, Point &point3)
        {
            this->polygonSize =3;
            points = new Point[3];
            points[0] = point1;
            points[1] = point2;
            points[2] = point3;
        }

};

class Sort
{
    public:
        Vertex *vertex;
        Sort(Vertex *v) : vertex(v) {}
        bool operator() (int index1, int index2)
        {
            if(vertex[index1].point.y > vertex[index2].point.y) return true;
            else if(vertex[index1].point.y == vertex[index2].point.y)
                if(vertex[index1].point.x > vertex[index2].point.x) return true;
            return false;
        }
};

class Edge
{
    public:
        mutable int index;
        Point startpoint;
        Point endpoint;

        bool operator< (const Edge L) const
        {
            if(L.startpoint.y == L.endpoint.y)
            {
                if(startpoint.y == endpoint.y)
                {
                    if(startpoint.y < L.startpoint.y) return true;
                    else return false;
                }
                if(IsConvex(startpoint,endpoint,L.startpoint)) return true;
                else return false;
            }
            else if(startpoint.y == endpoint.y)
            {
                if(IsConvex(L.startpoint,L.endpoint,startpoint)) return false;
                else return true;
            }
            else if(startpoint.y < L.startpoint.y)
            {
                if(IsConvex(L.startpoint,L.endpoint,startpoint)) return false;
                else return true;
            }
            else
            {
                if(IsConvex(startpoint,endpoint,L.startpoint)) return true;
                else return false;
            }
    }

    bool IsConvex(Point p1,Point p2, const Point p3) const
    {
        float tmp;
        tmp = (p3.y-p1.y)*(p2.x-p1.x)-(p3.x-p1.x)*(p2.y-p1.y);
        if(tmp>0) return 1;
        else return 0;
    }
};

class Monotone
{
    public:

    bool Convex(Point p1, Point p2, Point p3)
    {
        float tmp;
        tmp = (p3.y-p1.y)*(p2.x-p1.x)-(p3.x-p1.x)*(p2.y-p1.y);
        if(tmp>0) return 1;
        else return 0;
    }

    bool below(Point p1, Point p2)
    {
        if(p1.y < p2.y) return true;
        else if(p1.y == p2.y)
        {
            if(p1.x < p2.x) return true;
        }
        return false;
    }

    void TRIANGULATEMONOTONEPOLYGON(Polygon *Poly, list<Polygon> *triangles)
    {
        Point *points;
        int polygonSize;
        Polygon triangle;

        polygonSize=Poly->polygonSize;
        points = Poly->points;
        for(int i=0;i<polygonSize;i++)
        {
            cout<<(*Poly)[i].x<<" "<<(*Poly)[i].y<<endl;
        }
        cout<<endl<<endl;

        if(polygonSize == 3)
        {
            triangles->push_back(*Poly);
            return ;
        }

        int top = 0;
        int bottom=0;
        int i=0;
        for(i=1;i<polygonSize;i++)
        {
            if(below(points[i],points[bottom])) bottom = i;
            if(below(points[top],points[i])) top = i;
        }

        int *side = new int[polygonSize];
        int *serial = new int[polygonSize];

        serial[0] = top;
        side[top] = 0;
        int left = top+1;
        if(left>=polygonSize) left = 0;
        int right = top-1;
        if(right<0) right = polygonSize-1;
        for(i=1;i<(polygonSize-1);i++)
        {
           if(left==bottom)
            {
                serial[i] = right;
                right--;
                if(right<0) right = polygonSize-1;
                side[serial[i]] = Right;
            }
            else if(right==bottom)
            {
                serial[i] = left;
                left++;
                if(left>=polygonSize) left = 0;
                side[serial[i]] = Left;
            }
            else
            {
                if(below(points[left],points[right]))
                {
                    serial[i] = right;
                    right--;
                    if(right<0) right = polygonSize-1;
                    side[serial[i]] = Right;
                }
                else
                {
                    serial[i] = left;
                    left++;
                    if(left>=polygonSize) left = 0;
                    side[serial[i]] = Left;
                }
            }
        }
        serial[i] = bottom;
        side[bottom] = 0;

        int *S = new int[polygonSize];
        int Sptr = 0;

        S[0] = serial[0];
        S[1] = serial[1];

        int index=0;
        Sptr = 2;
        for(int i=2;i<(polygonSize-1);i++)
        {
            index = serial[i];
            if(side[index]!=side[S[Sptr-1]])
            {
                for(int j=0;j<(Sptr-1);j++)
                {
                    if(side[index]==Left) triangle.Triangle(points[S[j+1]],points[S[j]],points[index]);
                    else triangle.Triangle(points[S[j]],points[S[j+1]],points[index]);
                    triangles->push_back(triangle);
                }
                S[0] = serial[i-1];
                S[1] = serial[i];
                Sptr = 2;
            }
            else
            {
                Sptr--;
                while(Sptr>0)
                {
                    if(side[index]==Left)
                    {
                        if(Convex(points[index],points[S[Sptr-1]],points[S[Sptr]]))
                        {
                            triangle.Triangle(points[index],points[S[Sptr-1]],points[S[Sptr]]);
                            triangles->push_back(triangle);
                            Sptr--;
                        }
                        else
                            break;
                    }
                    else
                    {
                        if(Convex(points[index],points[S[Sptr]],points[S[Sptr-1]]))
                        {
                            triangle.Triangle(points[index],points[S[Sptr]],points[S[Sptr-1]]);
                            triangles->push_back(triangle);
                            Sptr--;
                        }
                        else
                            break;
                    }
                }
                Sptr++;
                S[Sptr] = index;
                Sptr++;
            }
        }
        index = serial[i];
        for(int j=0;j<(Sptr-1);j++)
        {
            if(side[S[j+1]]==Left) triangle.Triangle(points[S[j]],points[S[j+1]],points[index]);
            else triangle.Triangle(points[S[j+1]],points[S[j]],points[index]);
            triangles->push_back(triangle);
        }

        return ;
    }

    void AddDiagonal(Vertex *vertex, int *vertexsize, int *old1, int old2,
            int *type, std::set<Edge>::iterator *it,
            std::set<Edge> *T, int *helpers)
    {
        int new1,new2;
        new1 = *vertexsize;
        (*vertexsize)++;
        new2 = *vertexsize;
        ( *vertexsize)++;

        vertex[new1].point =vertex[*old1].point;
        vertex[new2].point = vertex[old2].point;
        //cout<<vertex[newindex1].p.x<<" "<<vertex[newindex1].p.y<<" "<<vertex[newindex2].p.x<<"  "<<vertex[newindex2].p.y<<endl;
        vertex[new2].next = vertex[old2].next;
        vertex[new1].next = vertex[*old1].next;

        vertex[vertex[old2].next].previous = new2;
        vertex[vertex[*old1].next].previous = new1;

        vertex[*old1].next = new2;
        vertex[old2].next = new1;
        vertex[new2].previous = *old1;
        vertex[new1].previous = old2;

        helpers[new1] = helpers[*old1];
        helpers[new2] = helpers[old2];
		type[new1] = type[*old1];
        type[new2] = type[old2];
        it[new1] = it[*old1];
        it[new2] = it[old2];
        if(it[new1] != T->end()) it[new1]->index = new1;
        if(it[new2] != T->end())it[new2]->index = new2;
    }

    void HANDLESTARTVERTEX(Edge *E,Vertex *v,Vertex *vertices,
                           int *index,pair<set<Edge>::iterator,bool> TRet,
                           set<Edge> *T,set<Edge>::iterator *Iterators,
                           int *helpers )
    {
        //cout<<"starttttttttttt"<<" "<<v->p.x<<" "<<v->p.y<<endl;
        E->startpoint = v->point;
        E->endpoint = vertices[v->next].point;
        E->index = *index;
        TRet = T->insert(*E);
        Iterators[*index] = TRet.first;
        helpers[*index] = *index;
        //cout<<"starttttttttttt"<<endl;
    }

    void HANDLEENDVERTEX(Vertex *v,Vertex *vertices,int *index,int *helpers,
                         set<Edge> *T,set<Edge>::iterator *Iterators,
                         int *newsize, int *type)
    {
        //cout<<"endddddddddddddd"<<" "<<v->p.x<<" "<<v->p.y<<endl;
        if(type[helpers[v->previous]]==MERGE)
        {
            AddDiagonal(vertices,newsize,index,helpers[v->previous],type, Iterators, T, helpers);
        }
        T->erase(Iterators[v->previous]);
        //cout<<"endddddddddddddd"<<endl;
    }

    void HANDLESPLITVERTEX(Edge *E,Vertex *v,Vertex *vertex,int *index,
                           set<Edge> *T, set<Edge>::iterator it, int *helpers,
                           int *newsize,int *type,set<Edge>::iterator *Iterators,
                           bool *error,int *index2, Vertex *v2,pair<set<Edge>::iterator,
                           bool>TRet )
    {
        //cout<<"splittttttttttt"<<" "<<v->p.x<<" "<<v->p.y<<endl;
        E->startpoint = v->point;
        E->endpoint = v->point;
        it = T->lower_bound(*E);
        if(it == T->begin())
        {
            *error = true;
            return;
        }
        it--;
        AddDiagonal(vertex,newsize,index,helpers[it->index],type, Iterators, T, helpers);
        *index2 = *newsize-2;
        v2 = &(vertex[*index2]);
        helpers[it->index] = *index;
        E->startpoint = v2->point;
        E->endpoint = vertex[v2->next].point;
        E->index = *index2;
        TRet = T->insert(*E);
        Iterators[*index2] = TRet.first;
        helpers[*index2] = *index2;
        //cout<<"splittttttttttt"<<endl;
    }

    void HANDLEMERGEVERTEX(Edge *E,Vertex *v,Vertex *vertex,int *type,
                           int *helpers,int *newsize,int *index,int *index2, Vertex *v2,
                           set<Edge> *T,set<Edge>::iterator *Iterators, bool *error,
                           set<Edge>::iterator it)
    {
        //cout<<"margeeeeeeeeeee"<<" "<<v->p.x<<" "<<v->p.y<<endl;
        if(type[helpers[v->previous]]==MERGE)
        {
            AddDiagonal(vertex,newsize,index,helpers[v->previous],type, Iterators, T, helpers);
            *index2 = *newsize-2;
            v2 = &(vertex[*index2]);
        }
        T->erase(Iterators[v->previous]);
        E->startpoint = v->point;
        E->endpoint = v->point;
        it = T->lower_bound(*E);
        if(it == T->begin())
        {
            *error = true;
            return;
        }
        it--;
        if(type[helpers[it->index]]==MERGE)
        {
            AddDiagonal(vertex,newsize,index2,helpers[it->index],type,Iterators, T, helpers);
        }
        helpers[it->index] = *index2;
        //cout<<"margeeeeeeeeeee"<<endl;
    }

    void HANDLEREGULARVERTEX(Edge *E,Vertex *v,Vertex *vertex,int *type,
                           int *helpers,int *newsize,int *index,int *index2, Vertex *v2,
                           set<Edge> *T,set<Edge>::iterator *Iterators,bool *error,
                           pair<set<Edge>::iterator,bool> TRet,set<Edge>::iterator it  )
    {
        //cout<<"regularrrrrrrrrrrr"<<" "<<v->p.x<<" "<<v->p.y<<endl;
        if(below(v->point,vertex[v->previous].point))
        {
            if(type[helpers[v->previous]]==MERGE)
            {
                AddDiagonal(vertex,newsize,index,helpers[v->previous],type, Iterators, T, helpers);
                *index2 = *newsize-2;
                v2 = &(vertex[*index2]);
            }
            T->erase(Iterators[v->previous]);
            E->startpoint = v2->point;
            E->endpoint = vertex[v2->next].point;
            E->index = *index2;
            TRet = T->insert(*E);
            Iterators[*index2] = TRet.first;
            helpers[*index2] = *index;
        }
        else
        {
            E->startpoint = v->point;
            E->endpoint = v->point;
            it = T->lower_bound(*E);
            if(it == T->begin())
            {
                *error = true;
                return;
            }
            it--;
            if(type[helpers[it->index]]==MERGE)
            {
                AddDiagonal(vertex,newsize,index,helpers[it->index],type, Iterators, T, helpers);
            }
            helpers[it->index] = *index;
        }
        //cout<<"regularrrrrrrrrrrr"<<endl;
    }

    void MAKEMONOTONE(Polygon *poly, list<Polygon> *monotone)
    {
        int Vertexsize = 0;
        Vertexsize=poly->polygonSize;
        int maxnum = Vertexsize*3;
        Vertex *vertex = new Vertex[maxnum];
        int newsize = Vertexsize;

        int startIndex = 0;
        int endIndex = startIndex + poly->polygonSize-1;
        for(int i=0;i<poly->polygonSize;i++)
        {
            vertex[i+startIndex].point = poly->GetPoint(i);
            if(i==0) vertex[i+startIndex].previous = endIndex;
            else vertex[i+startIndex].previous = i+startIndex-1;
            if(i==(poly->polygonSize-1)) vertex[i+startIndex].next = startIndex;
            else vertex[i+startIndex].next = i+startIndex+1;
        }
        startIndex = endIndex+1;

        int *priorityQ = new int [Vertexsize];
        for(int i=0;i<Vertexsize;i++) priorityQ[i] = i;
        std::sort(priorityQ,&(priorityQ[Vertexsize]),Sort(vertex));

        Vertex *v,*v2,*prev,*next;
        int *type = new int[maxnum];
        for(int i=0;i<Vertexsize;i++)
        {
            v = &(vertex[i]);
            prev = &(vertex[v->previous]);
            next = &(vertex[v->next]);

            if(below(prev->point,v->point)&&below(next->point,v->point))
            {
                if(Convex(next->point,prev->point,v->point)) type[i] = START;
                else type[i] = SPLIT;
            }
            else if(below(v->point,prev->point)&&below(v->point,next->point))
            {
                if(Convex(next->point,prev->point,v->point)) type[i] = END;
                else type[i] = MERGE;
            }
            else type[i] = REGULAR;
        }

        int *helpers = new int[maxnum];
        for(int i=0;i<maxnum;i++) helpers[i]=0;

        set<Edge> T;
        set<Edge>::iterator *Iterators,it;
        Iterators = new set<Edge>::iterator[maxnum];
        pair<set<Edge>::iterator,bool> TRet;
        for(int i = 0; i<Vertexsize; i++) Iterators[i] = T.end();


        int index,index2;
        Edge E;
        bool error = false;
  
        for(int i=0;i<Vertexsize;i++) 
		{
            index = priorityQ[i];
            v = &(vertex[index]);
            index2 = index;
            v2 = v;

            switch(type[index])
            {
                case START:
                    HANDLESTARTVERTEX(&E,v,vertex,&index,TRet,&T,Iterators,helpers);
                    break;

                case END:
                    HANDLEENDVERTEX(v,vertex,&index,helpers,&T,Iterators,&newsize,type);
                    break;

                case SPLIT:
                    HANDLESPLITVERTEX(&E,v,vertex,&index,&T,it,helpers,&newsize,
                                      type,Iterators,&error,&index2,v2,TRet);
                    break;

                case MERGE:
                    HANDLEMERGEVERTEX(&E,v,vertex,type,helpers,&newsize,&index,&index2,v2,
                                      &T,Iterators,&error,it);
                    break;

                case REGULAR:
                    HANDLEREGULARVERTEX(&E,v,vertex,type,helpers,&newsize,&index,&index2,v2,&T,
                                        Iterators,&error,TRet,it);
                    break;
            }

            if(error) break;
        }

        int *check = new int[newsize];
        memset(check,0,newsize*sizeof(int));

        if(!error) 
		{
            int Vertexsize;
            Polygon poly;
            for(int i=0;i<newsize;i++)
            {
                if(check[i]) continue;
                v = &(vertex[i]);
                next = &(vertex[v->next]);
                Vertexsize = 1;
                while(next!=v)
                {
                    next = &(vertex[next->next]);
                    Vertexsize++;
                }
                poly.polygonSize =Vertexsize;
                poly.points = new Point[Vertexsize];
                v = &(vertex[i]);
                poly[0] = v->point;
                next = &(vertex[v->next]);
                Vertexsize = 1;
                check[i] = 1;
                check[v->next] = 1;
                while(next!=v)
                {
                    poly[Vertexsize] = next->point;
                    check[next->next] = 1;
                    next = &(vertex[next->next]);
                    Vertexsize++;
                }
                monotone->push_back(poly);
            }
        }
        if(error) cout<<"Triangulation not possible"<<endl;
        return ;
    }

    void  Triangulate(Polygon *poly, list<Polygon> *triangles) {
        list<Polygon> monotonePolygon;
        list<Polygon>::iterator it;
        MAKEMONOTONE(poly,&monotonePolygon);
        cout<<endl<<"Monoton Polygon :"<<endl<<endl;
        for(it = monotonePolygon.begin(); it!=monotonePolygon.end();it++)
        {
            TRIANGULATEMONOTONEPOLYGON(&(*it),triangles);
        }
        return ;
    }
};


void output( list<Polygon> *polys)
{
	list<Polygon>::iterator iter;
	int s=polys->size();
    cout<<"Found Triangle : "<<s<<endl<<endl;
	for(iter = polys->begin(); iter != polys->end(); iter++)
    {
        Polygon *poly=&(*iter);
        int N;
        N=poly->polygonSize;
        for(int i=0;i<N;i++)
        {
            cout<<(*poly)[i].x<<" "<<(*poly)[i].y<<endl;
            if(i%3==2) cout<<endl;
        }
    }
}

void input(Polygon *poly)
{
	int N;
    float x,y;
    fin>>N;
    cout<<"Given Point : "<<N<<endl;
	poly->polygonSize =N;
    poly->points = new Point[N];
	for(int i=0;i<N;i++)
    {
		fin>>x;
        fin>>y;
        (*poly)[i].x = x;
		(*poly)[i].y = y;
	}
}



 Polygon polygon;
 list<Polygon> result;

void display(){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(BLACK, 0);	//color black
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(200,200,200,	0,0,0,	0,0,10);
	//gluLookAt(280,280,150,	0,0,0,	0,0,10);
	//gluLookAt(1500,1500,1500,	0,0,0,	0,0,10);
	//gluLookAt(50,50,50,	0,0,0,	0,0,10);
	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW); 
	
	glLineWidth(1.5); 
	glColor3f(1, 1, 1);

	glBegin(GL_LINES);{
		list<Polygon>::iterator iter;
	int s=result.size();
    for(iter = result.begin(); iter != result.end(); iter++)
    {
        Polygon *poly=&(*iter);
        int N,i;
        N=poly->polygonSize;
		for(i=0;i<N-1;i++)
        {
            glVertex3f((*poly)[i].x*3,(*poly)[i].y*3,0);
			glVertex3f((*poly)[i+1].x*3,(*poly)[i+1].y*3,0);
        }
		glVertex3f((*poly)[i].x*3,(*poly)[i].y*3,0);
		glVertex3f((*poly)[0].x*3,(*poly)[0].y*3,0);
    }
	}glEnd();

	int i;
	glLineWidth(2.5); 
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);{
		int N;
        N=polygon.polygonSize;
		for(i=0;i<N-1;i++){
			glVertex3f(polygon[i].x*3,polygon[i].y*3,0);
			glVertex3f(polygon[i+1].x*3,polygon[i+1].y*3,0);
	    }
		glVertex3f(polygon[i].x*3,polygon[i].y*3,0);
		glVertex3f(polygon[0].x*3,polygon[0].y*3,0);
	}glEnd();

	//ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
	glutSwapBuffers();
}


void animate(){
}

void init()
{
	glClearColor(BLACK, 0);

	/************************
	/ set-up projection here
	************************/
	//load the PROJECTION matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, 1, 0.1, 10000.0);
}

int main(int argc, char **argv)
{
	int N;
	fin>>N;
	for(int i=0;i<N;i++)
    {
        Monotone p;
        input(&polygon);
        p.Triangulate(&polygon,&result);
        output(&result);
        cout<<"----------------------------------------"<<endl;
    
		glutInit(&argc, argv);
		glutInitWindowSize(800, 600);
		glutInitWindowPosition(0, 0);
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);	//Depth, Double buffer, RGB color

		glutCreateWindow("Center Object");

		init();

		glEnable(GL_DEPTH_TEST);	//enable Depth Testing

		glutDisplayFunc(display);	//display callback function
		glutIdleFunc(animate);		//what you want to do in the idle time (when no drawing is occuring)

		glutMainLoop();		//The main loop of OpenGL
	}
    return 0;
}
