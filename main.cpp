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
	Vector operator+=(const Vector& a) {
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
Vector random_cos(const Vector& N) {
	double u1 = uniform(engine);
	double u2 = uniform(engine);
	double x = cos(2*M_PI*u1)*sqrt(1-u2);
	double y = sin(2*M_PI*u1)*sqrt(1-u2);
	double z = sqrt(u2);
	Vector T1;
	if (N[0] < N[1] && N[0] < N[2]) {
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

class Sphere {
public:
	explicit Sphere(const Vector& O, double R, const Vector& albedo, bool isMirror = false, bool isTransparent = false) : O(O), R(R), albedo(albedo), isMirror(isMirror), isTransparent(isTransparent) {
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
	Vector albedo;
	bool isMirror, isTransparent;
private :
	Vector O;
	double R;
};

class Scene {
public:
	Scene() {};
	bool intersect(const Ray& r, Vector& P, Vector& N, Vector& albedo, bool &mirror, bool &transparent, double &t)
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
			if (objects[i].intersect(r, localP, localN, localt) && localt<t)
			// regarde pour toutes les sphères de la scène, si elles intersectent le rayon r
			{
				t = localt;
				has_inter = true;
				albedo = objects[i].albedo;
				P = localP;
				N = localN;
				mirror = objects[i].isMirror;
				transparent = objects[i].isTransparent;
			}
		}
		return has_inter;
	}

	Vector get_color(const Ray& r, int rebond)
	{
	// renvoie la couleur du pixel
		if (rebond > 5) return Vector(0.,0.,0.);
		// on limite le nombre de rebond à 5 pour éviter que
		// le rayon soit réfléchi à l'infini entre 2 sphères

		Vector P, N, albedo;
		double t;
		bool mirror, transparent;
		bool inter = intersect(r, P, N, albedo, mirror, transparent, t);
		Vector color(0, 0, 0);    // noir 
		
		if (inter) 
		{
			if (mirror) 
			{
				Vector reflectedDir = r.u - 2*dot(r.u, N)*N;
				Ray reflectedRay(P + 0.001*N, reflectedDir);
				return get_color(reflectedRay, rebond + 1);
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
						return get_color(reflectedRay, rebond + 1);
					}
					Vector Tn = - sqrt(rad)*N2;
					Vector refractedDir = Tt + Tn;
					return get_color(Ray(P - 0.001*N2, refractedDir), rebond + 1);	
				}
				else
				// la sphère est diffuse 
				// éclairage direct
				{
					Vector PL = L-P;
					double d = sqrt(PL.carreNorm());    
					// distance entre le point d'intersection et la lumière
					Vector shadowP, shadowN, shadowAlbedo;
					double shadowt;
					bool shadowMirror, shadowTransp;
					Ray shadowRay(P + 0.001*N, PL/d);
					bool ombre = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowMirror, shadowTransp, shadowt);
					if (ombre && shadowt < d) {
						color = Vector(0., 0., 0.);
					} else {
						color = I / (4*M_PI*d*d) * albedo/M_PI *std::max(0., dot(N, PL/d));
					}

					// éclairage indirect
					Vector omega_i = random_cos(N);
					Ray omega_iRay(P + 0.001*N, omega_i);
					color += albedo*get_color(omega_iRay, rebond + 1);

				}	
			}
			return color;
		}
	}
	std::vector<Sphere> objects;
	Vector L;
	double I;

};

void integrateCos() {
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

void integrateCos4() {
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
	Sphere S1(Vector(0, 0, 0), 10, Vector(1, 1, 1));
	Sphere Ssol(Vector(0, -1000, 0), 990, Vector(1, 1, 1));
	Sphere Smur1(Vector(-1000, 0, 0), 940, Vector(1, 0., 0.));
	Sphere Smur2(Vector(1000, 0, 0), 940, Vector(0., 1, 0.));
	Sphere Smur3(Vector(0, 0, -1000), 940, Vector(0., 0., 1));
	Sphere Smur4(Vector(0, 0, 1000), 940, Vector(0., 1, 1));
	Sphere Splafond(Vector(0, 1000, 0), 940, Vector(1, 0.5, 0.));
	scene.objects.push_back(S1);
	scene.objects.push_back(Ssol);
	scene.objects.push_back(Smur1);
	scene.objects.push_back(Smur2);
	scene.objects.push_back(Smur3);
	scene.objects.push_back(Smur4);
	scene.objects.push_back(Splafond);

	double fov = 60 * M_PI /180;
	scene.I = 5E9;    // Intensité de la source lumineuse

	scene.L = Vector(-10, 20, 40);    // Position de la source lumineuse
	int nbrays = 10;

	std::vector<unsigned char> image(W*H * 3, 0);
#pragma omp parallel for schedule(dynamic, 1)
	// tableau dynamique que l'on remplit au fur et à mesure de la boucle
	// avec la couleur des pixels
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

				Vector u(j - W/2 + x2 + 0.5, i - H/2 + x1 + 0.5, - W / (2.*tan(fov/2)));    
				// direction du rayon sortant par le pixel i,j de la caméra
				u = u.get_normalized();
				Ray r(C, u);

				color += scene.get_color(r, 0);
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