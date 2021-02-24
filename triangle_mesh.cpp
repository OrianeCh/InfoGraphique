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
	explicit Vector(double x=0, double y=0, double z=0) 
	{
		coords[0] = x;
		coords[1] = y;
		coords[2] = z; 
	};
	double operator[](int i) const { return coords[i]; };
	double &operator[](int i) { return coords[i]; };
	double carreNorm() const    // renvoie le carré de la norme du vecteur
	{
		return coords[0]*coords[0] + coords[1]*coords[1] + coords[2]*coords[2];
	}
	Vector operator+=(const Vector& a) 
	{
		coords[0] += a[0];
		coords[1] += a[1];
		coords[2] += a[2];
		return *this;
	}
	Vector get_normalized()     // renvoie le vecteur normalisé
	{
		double n = sqrt(carreNorm());
		return Vector(coords[0] / n, coords[1] / n, coords[2] / n);
	}
private:
	double coords[3];
};
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
double dot(const Vector& a, const Vector& b) 
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
Vector cross(const Vector& a, const Vector& b)
{
	return Vector(a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]);
}
double carre(double x) 
{
	return x * x;
}
Vector random_cos(const Vector& N)
// Permet de générer un vecteur aléatoire pour la méthode de Monte Carlo
{
	double u1 = uniform(engine);
	double u2 = uniform(engine);
	double x = cos(2*M_PI*u1)*sqrt(1-u2);
	double y = sin(2*M_PI*u1)*sqrt(1-u2);
	double z = sqrt(u2);
	Vector T1;
	if (N[0] < N[1] && N[0] < N[2]) 
	{
		T1 = Vector(0, N[2], -N[1]);
	} else {
		if (N[1] < N[0] && N[1] < N[2]) {
			T1 = Vector(N[2], 0, -N[0]);
		} else {
			T1 = Vector(N[1], -N[0], 0);
		}
	}
	T1 = T1.get_normalized();
	Vector T2 = cross(N, T1);
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
	virtual bool intersect(const Ray& r, Vector& P, Vector& normale, double& t) = 0;
	Vector albedo;
	bool isMirror, isTransparent;
};

class BoundingBox {
public:
	bool intersect(const Ray& r)
	// Regarde les paires d'intersection du rayon avec les paires de plans de la boïte englobante
	// Regarde si l'intersection de ces intervalles est non-nulle
	{
		double t1x = (mini[0] - r.C[0])/r.u[0];
		double t2x = (maxi[0] - r.C[0])/r.u[0];
		double txMin = std::min(t1x, t2x);
		double txMax = std::max(t1x, t2x);

		double t1y = (mini[1] - r.C[1])/r.u[1];
		double t2y = (maxi[1] - r.C[1])/r.u[1];
		double tyMin = std::min(t1y, t2y);
		double tyMax = std::max(t1y, t2y);

		double t1z = (mini[2] - r.C[2])/r.u[2];
		double t2z = (maxi[2] - r.C[2])/r.u[2];
		double tzMin = std::min(t1z, t2z);
		double tzMax = std::max(t1z, t2z);

		double tMax = std::min(txMax, std::min(tyMax, tzMax));
		double tMin = std::max(txMin, std::max(tyMin, tzMin));   //intersection ici

		if (tMax < 0) return false;
		return tMax > tMin;

	}
	Vector mini, maxi;
};

class Noeud {
public :
	Noeud *fg, *fd;
	BoundingBox b;
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
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) {
		this -> albedo = albedo;
		isMirror = mirror;
		isTransparent = transparent;
		this->BVH = new Noeud;
	};

	BoundingBox buildBB(int debut, int fin) 
	// Construit la boïte englobante
	// debut et fin : indices de TRIANGLES
	{
		BoundingBox bb;
		bb.mini = Vector(1E9, 1E9, 1E9);
		bb.maxi = Vector(-1E9, -1E9, -1E9);
		for (int i = debut; i<fin; i++){
			for (int j = 0; j<3; j++) {
				bb.mini[j] = std::min(bb.mini[j], vertices[indices[i].vtxi][j]);
				bb.maxi[j] = std::max(bb.maxi[j], vertices[indices[i].vtxi][j]);
				bb.mini[j] = std::min(bb.mini[j], vertices[indices[i].vtxj][j]);
				bb.maxi[j] = std::max(bb.maxi[j], vertices[indices[i].vtxj][j]);
				bb.mini[j] = std::min(bb.mini[j], vertices[indices[i].vtxk][j]);
				bb.maxi[j] = std::max(bb.maxi[j], vertices[indices[i].vtxk][j]);
			}
		}
		return bb;
	};

	void buildBVH(Noeud* n, int debut, int fin) {
		n->debut = debut;
		n->fin = fin;
		n->b = buildBB(n->debut, n->fin);
		Vector diag = n->b.maxi - n->b.mini;
		int dim;
		if (diag[0] >= diag[1] && diag[0] >= diag[2]){
			dim = 0;
		} else {
			if (diag[1] >= diag[0] && diag[1] >= diag[2]){
				dim = 1;
			} else {
				if (diag[2] >= diag[0] && diag[2] >= diag[1]){
					dim = 2;
				}
			}
		}
		double milieu = (n->b.mini[dim] + n->b.maxi[dim])*0.5;
		int indice_pivot = n->debut;
		for (int i = n->debut; i < n->fin; i++) {
			double milieu_triangle = (vertices[indices[i].vtxi][dim] + vertices[indices[i].vtxj][dim] + vertices[indices[i].vtxk][dim])/3; 
			if (milieu_triangle < milieu) {
				std::swap(indices[i], indices[indice_pivot]);
				indice_pivot ++;
			}
		}
		n->fg = NULL;
		n->fd = NULL;
		if (indice_pivot == debut || indice_pivot == fin || (fin-debut < 5)) return;

		n->fg = new Noeud;
		n->fd = new Noeud;
		buildBVH(n->fg, n->debut, indice_pivot);
		buildBVH(n->fd, indice_pivot, n->fin);
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

	bool intersect(const Ray& r, Vector& P, Vector& normale, double& t) 
	// Regarde si le rayon intersecte le triangle
	{
		
		if (!BVH->b.intersect(r)) return false;

		t = 1E9;
		bool has_inter = false;

		std::list<Noeud*> l;
		l.push_back(BVH);
		while(!l.empty()) 
		{
			Noeud* c=l.front();
			l.pop_front();
			if (c->fg)
			{
				if (c->fg->b.intersect(r)) 
				{
				l.push_front(c->fg);
				}
				if (c->fd->b.intersect(r)) 
				{
				l.push_front(c->fd);
				};
			} else {
				for (int i = c->debut; i < c->fin; i++)
				{
					Vector &A = vertices[indices[i].vtxi];
					Vector &B = vertices[indices[i].vtxj];
					Vector &C = vertices[indices[i].vtxk];

					Vector e1 = B - A;
					Vector e2 = C - A;
					Vector N = cross(e1, e2);
					Vector AO = r.C - A;
					Vector AOu = cross(AO, r.u);
					double invUN = 1 / dot(r.u, N);
					double beta = -dot(e2, AOu)*invUN;
					double gamma = dot(e1, AOu)*invUN;
					double alpha = 1 - beta - gamma;
					double localt = -dot(AO, N)*invUN;
					if (beta >= 0 && gamma >= 0 && beta <= 1 && gamma <= 1 && alpha >= 0 && localt > 0) 
					{
						has_inter = true;
						if (localt < t) 
						{
							t = localt;
							normale = N.get_normalized();
							P = r.C + t * r.u;
						}
					}
				}
			}

		}
		return has_inter;	
}

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	//BoundingBox bb;

	Noeud* BVH;
	
};

class Sphere : public Object {
public:
	explicit Sphere(const Vector& O, double R, const Vector &albedo, bool isMirror = false, bool isTransparent = false) : O(O), R(R) {
		this -> albedo = albedo;
		this -> isMirror = isMirror;
		this -> isTransparent = isTransparent;
	};
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t)
	// renvoie true si le rayon r et la sphere s'intersectent
	// calcule le point d'intersection P et le vecteur normal N (normalisé) à la
	// sphère passant par ce point.
	{
		double a = 1;    // on suppose que la norme du vecteur u est 1
		double b = 2*dot(r.u, r.C - O);
		double c = (r.C - O).carreNorm() - R*R;
		double delta = b*b - 4*a*c;

		if (delta < 0) return false;    // on cherche une solution réelle

		double sqDelta = sqrt(delta);

		double t2 = (- b + sqDelta) / (2 * a);
		if (t2 < 0) return false;    // on cherche une solution positive

		double t1 = (- b - sqDelta) / (2 * a);
		if (t1 > 0)
			t = t1;    // on garde la solution la plus petite
		else 
			t = t2;

		P = r.C + t * r.u;    // point d'intersection
		N = (P - O).get_normalized();    // normale à la sphère passant par P

		return true;

	};
	Vector O;
	double R;
};

class Scene {
public:
	Scene() {};
	bool intersect(const Ray& r, Vector& P, Vector& N, Vector& albedo, bool &mirror, bool &transparent, double &t, int &objectid)
	// return true si il y a intersection avec au moins une sphere de la scene
	// donne le point P et la normale N de l'intersection la plus proche de l’origine du rayon 
	// parmi les intersections avec toutes les sphères de la scène
	{
		t = 1E10;
		bool has_inter = false;
		for (int i=0; i < objects.size(); i++)
		{
			Vector localP, localN;
			double localt;
			if (objects[i]->intersect(r, localP, localN, localt) && localt<t)
			// regarde pour toutes les sphères de la scène, si elles intersectent le rayon r
			{
				t = localt;
				has_inter = true;
				albedo = objects[i]->albedo;
				P = localP;
				N = localN;
				mirror = objects[i]->isMirror;
				transparent = objects[i]->isTransparent;
				objectid = i;
			}
		}
		return has_inter;
	}

	Vector get_color(const Ray& r, int rebond, bool lastDiffuse)
	{
	// renvoie la couleur du pixel
		if (rebond > 5) return Vector(0.,0.,0.);
		// on limite le nombre de rebond à 5 pour éviter que
		// le rayon soit réfléchi à l'infini entre 2 sphères

		Vector P, N, albedo;
		double t;
		bool mirror, transparent;
		int objectid;
		bool inter = intersect(r, P, N, albedo, mirror, transparent, t, objectid);
		Vector color(0, 0, 0);    // noir 
		
		if (inter) 
		{
			if (objectid == 0 )
			// Si la sphère est la source de lumière
			{
				if (rebond == 0 || !lastDiffuse) 
				{
					return Vector(I,I,I) / (4*M_PI*M_PI*(reinterpret_cast<Sphere*>(objects[0])->R)*(reinterpret_cast<Sphere*>(objects[0])->R));
				} else 
				{
					return Vector(0.,0.,0.);
				}
			}

			if (mirror) 
			{
				Vector reflectedDir = r.u - 2*dot(r.u, N)*N;
				Ray reflectedRay(P + 0.001*N, reflectedDir);
				return get_color(reflectedRay, rebond + 1, false);
			} 
			else 
			{
				if (transparent) 
				{
					Vector reflectedDir = r.u - 2*dot(r.u, N)*N;
					Ray reflectedRay(P + 0.001*N, reflectedDir);
					double n1 = 1, n2 = 1.4;
					Vector N2 = N;
					if (dot(r.u, N) > 0) 
					// si la normale ne pointe pas vers l'origine du rayon
					// on inverse les indices des milieux et on change la normale de direction
					{
						std::swap(n1, n2);
						N2 = -N;
					}
					Vector Tt = n1 / n2 * (r.u - dot(r.u, N2)*N2);
					double rad = 1 - carre(n1/n2) * (1 - carre(dot(r.u, N2)));
					if (rad < 0) 
					// Si le terme sous la racine est négatif, le rayon est réfléchi
					{
						Vector reflectedDir = r.u - 2*dot(r.u, N)*N;
						Ray reflectedRay(P + 0.001*N, reflectedDir);
						return get_color(reflectedRay, rebond + 1, false);
					}
					Vector Tn = - sqrt(rad)*N2;
					Vector refractedDir = Tt + Tn;
					return get_color(Ray(P - 0.001*N2, refractedDir), rebond + 1, false);	
				}
				else
				// la sphère est diffuse 
				{
					/*Vector PL = L-P;
					double d = sqrt(PL.carreNorm());    
					// distance entre le point d'intersection et la lumière
					Vector shadowP, shadowN, shadowAlbedo;
					double shadowt;
					bool shadowMirror, shadowTransp;
					Ray shadowRay(P + 0.001*N, PL/d);
					int objectsid;
					bool ombre = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowMirror, shadowTransp, shadowt, objectsid);
					if (ombre && shadowt < d) {
						color = Vector(0., 0., 0.);
					} else {
						color = I / (4*M_PI*d*d) * albedo/M_PI *std::max(0., dot(N, PL/d));
					}*/

					// éclairage direct
					Vector PL = L-P;
					PL = PL.get_normalized();
					Vector w = random_cos(-PL);
					Vector xprime = w*(reinterpret_cast<Sphere*>(objects[0])->R) + (reinterpret_cast<Sphere*>(objects[0])->O);
					Vector Pxprime = xprime - P;
					double d = sqrt(Pxprime.carreNorm());
					Pxprime = Pxprime/d;

					Vector shadowP, shadowN, shadowAlbedo;
					double shadowt;
					bool shadowMirror, shadowTransp;
					Ray shadowRay(P + 0.001*N, Pxprime);
					int objectsid;
					bool ombre = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowMirror, shadowTransp, shadowt, objectsid);
					if (ombre && shadowt < d - 0.002) 
					{
						color = Vector(0., 0., 0.);
					} else 
					{
						double carreR = carre(reinterpret_cast<Sphere*>(objects[0])->R);
						double proba = std::max(0., dot(-PL, w)) / (M_PI*carreR);
						double J = std::max(0., dot(w, -Pxprime)) / (d*d);
						color = I / (4*M_PI*M_PI*(carreR)) * albedo/M_PI *std::max(0., dot(N, Pxprime)) * J/proba;
					}

					// éclairage indirect
					Vector omega_i = random_cos(N);
					Ray omega_iRay(P + 0.001*N, omega_i);
					color += albedo*get_color(omega_iRay, rebond + 1, true);

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
	int W = 512;
	int H = 512;

	/*integrateCos();
	return 0;*/

	Vector C(0, 0, 55);    // centre de la caméra
	Scene scene;

	scene.I = 5E9;    // Intensité de la source lumineuse
	scene.L = Vector(-10, 20, 40);    // Position de la source lumineuse

	Sphere Ssource(scene.L, 1, Vector(1, 1, 1));
	Sphere S1(Vector(0, 0, 0), 10, Vector(1, 1, 1));
	Sphere S2(Vector(-10, 0, -20), 10, Vector(1, 1, 1));
	Sphere S3(Vector(10, 0, 20), 10, Vector(1, 1, 1));
	Sphere Ssol(Vector(0, -1000, 0), 990, Vector(1, 1, 1));
	Sphere Smur1(Vector(-1000, 0, 0), 940, Vector(1, 0., 0.));
	Sphere Smur2(Vector(1000, 0, 0), 940, Vector(0., 1, 0.));
	Sphere Smur3(Vector(0, 0, -1000), 940, Vector(0., 0., 1));
	Sphere Smur4(Vector(0, 0, 1000), 940, Vector(0., 1, 1));
	Sphere Splafond(Vector(0, 1000, 0), 940, Vector(1, 0.5, 0.));
	TriangleMesh m(Vector(1., 1., 1.));
	m.readOBJ("13463_Australian_Cattle_Dog_v3.obj");
	double theta = 45./360.*2*M_PI;
	for (int i=0; i < m.vertices.size(); i++)
	// on remet le chien dans le bon sens et bien positionnée dans l'image
	{
		std::swap(m.vertices[i][1], m.vertices[i][2]);
		m.vertices[i][2] = - m.vertices[i][2];
		m.vertices[i][2] += 5;
		m.vertices[i][1] -= 10;
		m.vertices[i][0] = cos(theta)*m.vertices[i][0] - sin(theta)*m.vertices[i][2];
        m.vertices[i][2] = sin(theta)*m.vertices[i][0] + cos(theta)*m.vertices[i][2];
	}
	for (int i=0; i < m.normals.size(); i++){
		std::swap(m.normals[i][1], m.normals[i][2]);
	}
	m.buildBVH(m.BVH, 0, m.indices.size());
	//m.buildBB();

	scene.objects.push_back(&Ssource);
	//scene.objects.push_back(&S1);
	//scene.objects.push_back(&S2);
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

	std::vector<unsigned char> image(W*H * 3, 0);
	// tableau dynamique que l'on remplit au fur et à mesure de la boucle
	// avec la couleur des pixels

#pragma omp parallel for schedule(dynamic, 1)
	// parallélisation pour gagner du temps
	for (int i = 0; i < H; i++) 
	{
		for (int j = 0; j < W; j++) 
		{
			Vector u(j - W/2, i - H/2, - W / (2.*tan(fov/2)));    
			// direction du rayon sortant par le pixel i,j de la caméra
			u = u.get_normalized();
			Ray r(C, u);

			Vector color(0., 0., 0.);
			for (int k= 0; k<nbrays; k++) {
				
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

				// Prise en compte de la profondeur de champs
				Vector target = C + 55*u;
				Vector Cprime = C + Vector(x3, x4, 0);
				Vector uprime = (target - Cprime).get_normalized();

				Ray r(Cprime, uprime);

				color += scene.get_color(r, 0, false);
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