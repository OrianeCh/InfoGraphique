#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <random>
static std::default_random_engine engine(10) ; // random seed = 10 
static std::uniform_real_distribution<double> uniform(0, 1);

#include <iostream>
#include <list>

#define M_PI 3.14159265

class Vector {
public :
	explicit Vector(double x=0., double y=0., double z=0.) 
	{
		coordinates[0] = x;
		coordinates[1] = y;
		coordinates[2] = z; 
	};
	double operator[](int i) const { return coordinates[i]; };
	double &operator[](int i) { return coordinates[i]; };
private:
	double coordinates[3];
};

Vector operator+=(Vector& a, const Vector& b)
{
	return Vector(a[0] += b[0], a[1] += b[1], a[2] += b[2]);
}
Vector operator+=(const Vector& a, Vector& b)
{
	return Vector(b[0] += a[0], b[1] += a[1], b[2] += a[2]);
}
Vector operator+(const Vector& a, const Vector& b) 
{
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator*(double a, const Vector& b) 
{
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, double b)
{
	return Vector(b*a[0], b*a[1], b*a[2]);
}
Vector operator*(const Vector& a, const Vector& b)
{
	return Vector(b[0]*a[0], b[1]*a[1], b[2]*a[2]);
}
Vector operator/(const Vector& a, double b)
{
	return Vector(a[0]/b, a[1]/b, a[2]/b);
}
Vector operator/(double a, const Vector& b)
{
	return Vector(b[0]/a, b[1]/a, b[2]/a);
}
Vector operator-(const Vector& a, const Vector& b) 
{
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator-(const Vector& a) 
{
	return Vector(- a[0], - a[1], - a[2]);
}
double scalar(const Vector& a, const Vector& b) 
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
Vector vectorial(const Vector& a, const Vector& b)
{
	return Vector(a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]);
}
double square(double x) 
{
	return x * x;
}
double carreNorm(const Vector& a)    // renvoie le carré de la norme du vecteur
{
	return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}
double Norm(const Vector& a)
{
	return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}
Vector normalized(const Vector& a)     // renvoie le vecteur normalisé
{
	double n = sqrt(carreNorm(a));
	return Vector(a[0] / n, a[1] / n, a[2] / n);
}
Vector random_dir(const Vector& N)
// Permet de générer un vecteur aléatoire pour la méthode de Monte Carlo
{
	double u1 = uniform(engine);
	double u2 = uniform(engine);
	double x = cos(2*M_PI*u1)*sqrt(1-u2);
	double y = sin(2*M_PI*u1)*sqrt(1-u2);
	double z = sqrt(u2);
	Vector T1 = Vector(N[1], -N[0], 0);
	if (N[0] < N[1] && N[0] < N[2]) 
	{
		T1 = Vector(0, N[2], -N[1]);
	} 
	if (N[1] < N[0] && N[1] < N[2]) 
	{
		T1 = Vector(N[2], 0, -N[0]);
	} 
	T1 = normalized(T1);
	Vector T2 = vectorial(N, T1);
	return z*N + x*T1 + y*T2;
}

class Ray {
public:
	explicit Ray(const Vector& C, const Vector& u) : C(C), u(u) {
	};
	Vector C, u;
};

class Object {
public :
	Object() {};
	virtual bool intersect(const Ray& r, Vector& P, Vector& normal, double& t, Vector &color) = 0;
	Vector albedo;
	bool mirror, transparent;
};

class BoundingBox {
public:
	Vector min_vertice, max_vertice;
	bool intersect(const Ray& r)
	// Regarde les paires d'intersection du rayon avec les paires de plans de la boïte englobante
	// Regarde si l'intersection de ces intervalles est non-nulle
	{
		bool intersection = false;
		double t_x_min = std::min((min_vertice[0] - r.C[0])/r.u[0], (max_vertice[0] - r.C[0])/r.u[0]);
		double t_x_max = std::max((min_vertice[0] - r.C[0])/r.u[0], (max_vertice[0] - r.C[0])/r.u[0]);

		double t_y_min = std::min((min_vertice[1] - r.C[1])/r.u[1], (max_vertice[1] - r.C[1])/r.u[1]);
		double t_y_max = std::max((min_vertice[1] - r.C[1])/r.u[1], (max_vertice[1] - r.C[1])/r.u[1]);

		double t_z_min = std::min((min_vertice[2] - r.C[2])/r.u[2], (max_vertice[2] - r.C[2])/r.u[2]);
		double t_z_max = std::max((min_vertice[2] - r.C[2])/r.u[2], (max_vertice[2] - r.C[2])/r.u[2]);

		double rightInterval = std::min(t_x_max, std::min(t_y_max, t_z_max));
		double leftInterval = std::max(t_x_min, std::max(t_y_min, t_z_min));   

		if (leftInterval < 0 || rightInterval < 0) return false;
		if (rightInterval > leftInterval) 
		//l'intervalle est non-vide
		{
			intersection = true;
		}
		return intersection;   

	}
};

class Noeud {
public :
	Noeud *fg, *fd;
	BoundingBox bounding_box;
	int debut, fin;
};

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class TriangleMesh : public Object {
public:
  ~TriangleMesh() {}
	TriangleMesh(const Vector& albedo, bool mir = false, bool transp = false) {
		this->albedo = albedo;
		this->mirror = mir;
		this->transparent = transp;
		this->racineBVH = new Noeud;
	};

	BoundingBox newBBox(int debut, int fin) 
	// Construit la boîte englobante
	// debut et fin : indices de TRIANGLES
	{
		BoundingBox bbox;
		bbox.min_vertice = Vector(1E5, 1E5, 1E5);
		bbox.max_vertice = Vector(-1E5, -1E5, -1E5);
		for (int i = debut; i<fin; i++)
		{
			for (int j = 0; j<3; j++) 
			{
				bbox.min_vertice[j] = std::min(bbox.min_vertice[j], std::min(vertices[indices[i].vtxi][j], 
									  std::min(vertices[indices[i].vtxj][j], vertices[indices[i].vtxk][j])));
				bbox.max_vertice[j] = std::max(bbox.max_vertice[j], std::max(vertices[indices[i].vtxi][j], 
									  std::max(vertices[indices[i].vtxj][j], vertices[indices[i].vtxk][j])));
			}
		}
		return bbox;
	};

	int newBVH(Noeud* n, int debut, int fin) {
		n->debut = debut;
		n->fin = fin;
		n->bounding_box = newBBox(n->debut, n->fin);
		Vector diag = n->bounding_box.max_vertice - n->bounding_box.min_vertice;
		int dirSeparation = 0;
		if (diag[1] >= diag[0] && diag[1] >= diag[2])
		{
			dirSeparation = 1;
		} 
		if (diag[2] >= diag[0] && diag[2] >= diag[1])
		{
		 	dirSeparation = 2;
		}
		
		int indice_pivot = n->debut;
		double milieu = (n->bounding_box.min_vertice[dirSeparation] + n->bounding_box.max_vertice[dirSeparation])/2;
		for (int i = n->debut; i < n->fin; i++) 
		{
			double centre = (vertices[indices[i].vtxi][dirSeparation] + vertices[indices[i].vtxj][dirSeparation] + vertices[indices[i].vtxk][dirSeparation])/3; 
			if (centre < milieu) 
			{
				std::swap(indices[i], indices[indice_pivot]);
				indice_pivot ++;
			}
		}
		n->fg = NULL;
		n->fd = NULL;
		if (indice_pivot == debut || indice_pivot == fin || (fin-debut < 5)) return 0;

		n->fg = new Noeud;
		n->fd = new Noeud;
		newBVH(n->fg, n->debut, indice_pivot);
		newBVH(n->fd, indice_pivot, n->fin);
	}

	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				} else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						} else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					} else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							} else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								} else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);

	}

	bool intersect(const Ray& r, Vector& P, Vector& normal, double& t, Vector &color) 
	// Regarde si le rayon intersecte le triangle
	{
		
		if (!racineBVH->bounding_box.intersect(r)) return false;

		t = 1E5;
		bool intersection = false;

		std::list<Noeud*> l;
		l.push_back(racineBVH);
		while(!l.empty()) 
		{
			Noeud* courant = l.front();
			l.pop_front();
			if (courant->fg)
			{
				if (courant->fg->bounding_box.intersect(r)) 
				{
				l.push_front(courant->fg);
				}
				if (courant->fd->bounding_box.intersect(r)) 
				{
				l.push_front(courant->fd);
				};
			} else {
				for (int i = courant->debut; i < courant->fin; i++)
				{
					Vector &A = vertices[indices[i].vtxi];
					Vector &B = vertices[indices[i].vtxj];
					Vector &C = vertices[indices[i].vtxk];

					Vector e1 = B - A;
					Vector e2 = C - A;
					Vector N = vectorial(e1, e2);
					double beta = -scalar(e2, vectorial(r.C - A, r.u))/scalar(r.u, N);
					double gamma = scalar(e1, vectorial(r.C - A, r.u))/scalar(r.u, N);
					double alpha = 1 - beta - gamma;
					double localt = -scalar(r.C - A, N)/scalar(r.u, N);
					if (beta >= 0 && gamma >= 0 && alpha >= 0 && beta <= 1 && gamma <= 1  && localt > 0) 
					{
						intersection = true;
						if (localt < t) 
						{
							t = localt;
							P = r.C + t * r.u;

							int Htex = Htexture[indices[i].group];
							int Wtex = Wtexture[indices[i].group];

							normal = alpha*normals[indices[i].ni] + beta*normals[indices[i].nj] + gamma*normals[indices[i].nk];
							
							Vector UV = alpha*uvs[indices[i].uvi] + beta*uvs[indices[i].uvj] + gamma*uvs[indices[i].uvk];
							UV = UV * Vector(Wtex, Htex, 0);
							int uvx = UV[0] + 0.5;
							int uvy = UV[1] + 0.5;
							uvx = uvx % Wtex;
							uvy = uvy % Htex;
							if (uvx < 0) uvx += Wtex;
							if (uvy < 0) uvy += Htex;
							uvy = Htex - uvy - 1;

							color = Vector(std::pow(textures[indices[i].group][(uvy*Wtex + uvx)*3] / 255., 2.2), std::pow(textures[indices[i].group][(uvy*Wtex + uvx)*3 + 1]/ 255., 2.2), std::pow(textures[indices[i].group][(uvy*Wtex + uvx)*3 + 2]/255.,2.2));
						}
					}
				}
			}

		}
		normal = normalized(normal);
		return intersection;	
	}

	void loadTexture(const char* filename) {
		int W, H, C;
		unsigned char* texture = stbi_load(filename,&W, &H, &C, 3);
		Wtexture.push_back(W);
		Htexture.push_back(H);
		textures.push_back(texture);
	}

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	std::vector<unsigned char *> textures;
	std::vector<int> Wtexture, Htexture;

	Noeud* racineBVH;
	
};

class Sphere : public Object {
public:
	explicit Sphere(const Vector& O, double R, const Vector& albedo, bool mirror = false, bool transparent = false) : O(O), R(R) {
		this->albedo = albedo;
		this->mirror = mirror;
		this->transparent = transparent;
	};
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t, Vector& color)
	// renvoie true si le rayon r et la sphere s'intersectent
	// calcule le point d'intersection P et le vecteur normal N (normalisé) à la
	// sphère passant par ce point.
	{
		double a = 1;    // on suppose que la norme du vecteur u est 1
		double b = 2*scalar(r.u, r.C - O);
		double c = carreNorm(r.C - O) - R*R;
		double delta = b*b - 4*a*c;

		if (delta < 0) return false;    // on cherche une solution réelle

		double t1 = (- b - sqrt(delta)) / (2 * a);
		double t2 = (- b + sqrt(delta)) / (2 * a);

		if (t2 < 0) return false;    // on cherche une solution positive

		if (t1 > 0)
			t = t1;    // on garde la solution la plus petite
		else 
			t = t2;

		P = r.C + t * r.u;    // point d'intersection
		N = normalized(P-O);    // normale à la sphère passant par P
		color = this->albedo;

		return true;

	};
	Vector O;
	double R;
};

class Scene {
public:
	Scene() {};
	bool intersect(const Ray& r, Vector& P, Vector& N, Vector& albedo, bool& mir, bool& transp, double& t, int& objectid)
	// return true si il y a intersection avec au moins une sphere de la scene
	// donne le point P et la normale N de l'intersection la plus proche de l’origine du rayon 
	// parmi les intersections avec toutes les sphères de la scène
	{
		t = 1E5;
		bool intersection = false;
		for (int i=0; i < objects.size(); i++)
		{
			Vector localP, localN, localalbedo;
			double localt;
			if (objects[i]->intersect(r, localP, localN, localt, localalbedo) && localt<t)
			// regarde pour toutes les sphères de la scène, si elles intersectent le rayon r
			{
				intersection = true;
				P = localP;
				N = localN;
				albedo = localalbedo;
				mir = objects[i]->mirror;
				transp = objects[i]->transparent;
				t = localt;
				objectid = i;
			}
		}
		return intersection;
	}

	Vector get_color(const Ray& r, int rebond, bool lastDiffuse)
	{
	// renvoie la couleur du pixel
		if (rebond > 5) return Vector(0.,0.,0.);
		// on limite le nombre de rebond à 5 pour éviter que
		// le rayon soit réfléchi à l'infini entre 2 sphères

		Vector P, N, albedo;
		bool mir, transp;
		double t;
		int objectid;
		bool intersection = intersect(r, P, N, albedo, mir, transp, t, objectid);
		Vector color(0., 0., 0.);    // noir 
		Ray reflectedRay(P + 0.001*N, r.u - 2*scalar(r.u, N)*N);

		if (intersection) 
		{
			if (objectid == 0 )
			// Si la sphère est la source de lumière
			{
				if (rebond == 0 || !lastDiffuse) 
				{
					return Vector(I,I,I) / (4*M_PI*M_PI*(reinterpret_cast<Sphere*>(objects[0])->R)*(reinterpret_cast<Sphere*>(objects[0])->R));
				} else 
				{
					return color;
				}
			}

			if (mir) 
			{
				return get_color(reflectedRay, rebond + 1, false);
			} 
			else 
			{
				if (transp) 
				{
					double n1 = 1;
					double n2 = 1.4;
					Vector Norigine = N;
					if (scalar(r.u, N) > 0) 
					// si la normale ne pointe pas vers l'origine du rayon
					// on inverse les indices des milieux et on change la normale de direction
					{
						std::swap(n1, n2);
						Norigine = -N;
					}
					double sousRacine = 1 - square(n1/n2) * (1 - square(scalar(r.u, Norigine)));
					if (sousRacine < 0) 
					// Si le terme sous la racine est négatif, le rayon est réfléchi
					{
						return get_color(reflectedRay, rebond + 1, false);
					}
					Vector Tt = n1 / n2 * (r.u - scalar(r.u, Norigine)*Norigine);
					Vector Tn = - sqrt(sousRacine)*Norigine;
					return get_color(Ray(P - 0.001*Norigine, Tt + Tn), rebond + 1, false);	
				}
				else
				// la sphère est diffuse 
				{
					// éclairage direct
					Vector PL = L-P;
					PL = normalized(PL);
					Vector w = random_dir(-PL);
					Vector xprime = w*(reinterpret_cast<Sphere*>(objects[0])->R) + (reinterpret_cast<Sphere*>(objects[0])->O);
					Vector Pxprime = xprime - P;
					double norme = Norm(Pxprime);
					Pxprime = Pxprime/norme;

					Vector ombreP, ombreN, ombre_albedo;
					double ombret;
					bool ombre_mir, ombre_transp;
					Ray ombrer(P + 0.001*N, Pxprime);
					int objectsid;
					bool ombre = intersect(ombrer, ombreP, ombreN, ombre_albedo, ombre_mir, ombre_transp, ombret, objectsid);

					if (!ombre || ombret >= norme - 0.002)
					{
						double carreR = square(reinterpret_cast<Sphere*>(objects[0])->R);
						double proba = std::max(0., scalar(-PL, w)) / (M_PI*carreR);
						double J = std::max(0., scalar(w, -Pxprime)) / (norme*norme);
						color = I / (4*M_PI*M_PI*(carreR)) * albedo/M_PI *std::max(0., scalar(N, Pxprime)) * J/proba;
					}

					// éclairage indirect
					Vector omega_i = random_dir(N);
					Ray omega_ir(P + 0.001*N, omega_i);
					color += albedo*get_color(omega_ir, rebond + 1, true);

				}	
			}
			return color;
		}
	}
	std::vector<Object*> objects;
	Vector L;
	double I;

};

void integrateCos() 
// fonction d'entrainement à la méthode de Monte-Carlo, non-utilisée dans le raytracer
{
    int N = 10000;
    double sigma = 0.25;
    double s = 0;
    for (int i = 0; i < N; i++) {
        double u1 = uniform(engine);
        double u2 = uniform(engine);
        double xi = sigma*cos(2*M_PI*u1)*sqrt(-2*log(u2));
		if ((xi > -M_PI/2) && (xi < M_PI/2)) {
        	double p = 1/(sigma*sqrt(2 * M_PI)) * exp(-xi*xi / (2*sigma*sigma));
        	s += pow(cos(xi), 10)/p/N;
		}
    }
    std::cout <<s << std::endl;
}

void integrateCos4() 
// fonction d'entrainement à la méthode de Monte-Carlo, non-utilisée dans le raytracer
{
    int N = 1000000;
    double sigma = 1;
    double s = 0;
    for (int i = 0; i < N; i++) {
        double u1 = uniform(engine);
        double u2 = uniform(engine);
		double u3 = uniform(engine);
		double u4 = uniform(engine);
        double x1 = sigma*cos(2*M_PI*u1)*sqrt(-2*log(u2));
		double x2 = sigma*sin(2*M_PI*u1)*sqrt(-2*log(u2));
		double x3 = sigma*cos(2*M_PI*u3)*sqrt(-2*log(u4));
		double x4 = sigma*sin(2*M_PI*u3)*sqrt(-2*log(u4));
		if (((x1 > -M_PI/2) && (x1 < M_PI/2)) && ((x2 > -M_PI/2) && (x2 < M_PI/2)) && ((x3 > -M_PI/2) && (x3 < M_PI/2)) && ((x4 > -M_PI/2) && (x4 < M_PI/2))) {
        	double p1 = 1/(sigma*sqrt(2 * M_PI)) * exp(-x1*x1 / (2*sigma*sigma));
			double p2 = 1/(sigma*sqrt(2 * M_PI)) * exp(-x2*x2 / (2*sigma*sigma));
			double p3 = 1/(sigma*sqrt(2 * M_PI)) * exp(-x3*x3 / (2*sigma*sigma));
			double p4 = 1/(sigma*sqrt(2 * M_PI)) * exp(-x4*x4 / (2*sigma*sigma));
        	s += pow(cos(x1+x2+x3+x4), 2)/(p1*p2*p3*p4)/N;
		}
    }
    std::cout <<s << std::endl;
}

int main() {
	int W = 256;
	int H = 256;

	/*integrateCos();
	return 0;*/

	Vector C(0., 0., 55.);    // centre de la caméra
	Scene scene;

	scene.I = 4E9;    // Intensité de la source lumineuse
	scene.L = Vector(-10, 20, 40);    // Position de la source lumineuse

	Sphere Ssource(scene.L, 1, Vector(1, 1, 1));
	Sphere S1(Vector(0, 0, 0), 10, Vector(1, 1, 1));
	Sphere S2(Vector(-25, 0, 20), 15, Vector(1, 1, 1), true);
	Sphere S3(Vector(10, 0, 20), 10, Vector(1, 1, 1));
	Sphere Ssol(Vector(0, -1000, 0), 990, Vector(0.1, 0.1, 0.1));
	Sphere Smur1(Vector(-1000, 0, 0), 940, Vector(0.9, 0.9, 0.2));
	Sphere Smur2(Vector(1000, 0, 0), 940, Vector(1, 1, 1));
	Sphere Smur3(Vector(0, 0, -1000), 940, Vector(1, 1, 1));
	Sphere Smur4(Vector(0, 0, 1000), 940, Vector(1, 1, 1));
	Sphere Splafond(Vector(0, 1000, 0), 940, Vector(0.8, 0.8, 0.8));
	TriangleMesh m(Vector(1., 1., 1.));
	m.readOBJ("13463_Australian_Cattle_Dog_v3.obj");
	m.loadTexture("Australian_Cattle_Dog_dif.jpg");
	double theta = 45./360.*2*M_PI;
	for (int i=0; i < m.vertices.size(); i++)
	// on remet le chien dans le bon sens et bien positionné dans l'image
	{
		std::swap(m.vertices[i][1], m.vertices[i][2]);
		m.vertices[i][2] = - m.vertices[i][2];
		m.vertices[i][2] -= 10;
		m.vertices[i][1] -= 10;
		m.vertices[i][0] += 10;
		double old_vertices = m.vertices[i][0];
		m.vertices[i][0] = cos(theta)*m.vertices[i][0] - sin(theta)*m.vertices[i][2];
        m.vertices[i][2] = sin(theta)*old_vertices + cos(theta)*m.vertices[i][2];
		
	}
	for (int i=0; i < m.normals.size(); i++){
		std::swap(m.normals[i][1], m.normals[i][2]);
		m.normals[i][2] = - m.normals[i][2];
		double old_normals = m.normals[i][0];
		m.normals[i][0] = cos(theta)*m.normals[i][0] - sin(theta)*m.normals[i][2];
        m.normals[i][2] = sin(theta)*old_normals + cos(theta)*m.normals[i][2];

	}
	m.newBVH(m.racineBVH, 0, m.indices.size());

	scene.objects.push_back(&Ssource);
	//scene.objects.push_back(&S1);
	scene.objects.push_back(&S2);
	//scene.objects.push_back(&S3);
	scene.objects.push_back(&m);
	scene.objects.push_back(&Ssol);
	scene.objects.push_back(&Smur1);
	scene.objects.push_back(&Smur2);
	scene.objects.push_back(&Smur3);
	scene.objects.push_back(&Smur4);
	scene.objects.push_back(&Splafond);

	double fov = 60 * M_PI /180;

	int nbrays = 100;

	/*Vector up(0., cos(20*M_PI/180), sin(20*M_PI/180));
	Vector right(1, 0, 0);
	Vector viewDirection = vectorial(up, right);*/

	std::vector<unsigned char> image(W*H * 3, 0);
	// tableau dynamique que l'on remplit au fur et à mesure de la boucle
	// avec la couleur des pixels

#pragma omp parallel for schedule(dynamic, 1)
	// parallélisation pour gagner du temps
	for (int i = 0; i < H; i++) 
	{
		for (int j = 0; j < W; j++) 
		{  
			Vector color;
			for (int k= 0; k<nbrays; k++) 
			{
				double u1 = uniform(engine);
				double u2 = uniform(engine);
				double x1 = 0.25*cos(2*M_PI*u1)*sqrt(-2*log(u2));
				double x2 = 0.25*cos(2*M_PI*u1)*sqrt(-2*log(u2));

				u1 = uniform(engine);
				u2 = uniform(engine);
				double x3 = 0.000001*cos(2*M_PI*u1)*sqrt(-2*log(u2));
				double x4 = 0.000001*cos(2*M_PI*u1)*sqrt(-2*log(u2));

				Vector u(j - W/2 + x2 + 0.5, i - H/2 + x1 + 0.5, - W / (2.*tan(fov/2)));    
				// direction du rayon sortant par le pixel i,j de la caméra

				u = normalized(u);
				//u = u[0] * right + u[1] * up + viewDirection;

				// Prise en compte de la profondeur de champs
				Vector P = C + 55*u;
				Vector Cprime = C + Vector(x3, x4, 0);
				Vector uprime = normalized(P - Cprime);

				Ray rprime(Cprime, uprime);

				color += scene.get_color(rprime, 0, false);
			}
			color = color / nbrays;
			

			image[((H-i-1)*W + j) * 3 + 0] = std::min(255., pow(color[0], 0.45));
			image[((H-i-1)*W + j) * 3 + 1] = std::min(255., pow(color[1], 0.45));
			image[((H-i-1)*W + j) * 3 + 2] = std::min(255., pow(color[2], 0.45));
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}