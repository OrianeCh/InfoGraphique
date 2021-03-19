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

class Sphere {
public:
	explicit Sphere(const Vector& O, double R, const Vector& albedo, bool mirror = false, bool transparent = false) : O(O), R(R), albedo(albedo), mirror(mirror), transparent(transparent) {
	};
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t)
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

		return true;

	};
	Vector albedo;
	bool mirror, transparent;
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
			Vector localP, localN;
			double localt;
			if (objects[i].intersect(r, localP, localN, localt) && localt<t)
			// regarde pour toutes les sphères de la scène, si elles intersectent le rayon r
			{
				
				intersection = true;
				P = localP;
				N = localN;
				albedo = objects[i].albedo;
				mir = objects[i].mirror;
				transp = objects[i].transparent;
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
					return Vector(I,I,I) / (4*M_PI*M_PI*objects[0].R*objects[0].R);
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
					Vector xprime = w*objects[0].R + objects[0].O;
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
						double proba = std::max(0., scalar(-PL, w)) / (M_PI*objects[0].R*objects[0].R);
						double J = std::max(0., scalar(w, -Pxprime)) / (norme*norme);
						color = I / (4*M_PI*M_PI*objects[0].R*objects[0].R) * albedo/M_PI *std::max(0., scalar(N, Pxprime)) * J/proba;
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
	std::vector<Sphere> objects;
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
	scene.objects.push_back(Ssource);
	scene.objects.push_back(S1);
	//scene.objects.push_back(S2);
	//scene.objects.push_back(S3);
	scene.objects.push_back(Ssol);
	scene.objects.push_back(Smur1);
	scene.objects.push_back(Smur2);
	scene.objects.push_back(Smur3);
	scene.objects.push_back(Smur4);
	scene.objects.push_back(Splafond);

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

			Vector color;
			for (int k= 0; k<nbrays; k++) {
				
				double u1 = uniform(engine);
				double u2 = uniform(engine);
				double x1 = 0.25*cos(2*M_PI*u1)*sqrt(-2*log(u2));
				double x2 = 0.25*cos(2*M_PI*u1)*sqrt(-2*log(u2));

				u1 = uniform(engine);
				u2 = uniform(engine);
				double x3 = 1*cos(2*M_PI*u1)*sqrt(-2*log(u2));
				double x4 = 1*cos(2*M_PI*u1)*sqrt(-2*log(u2));

				Vector u(j - W/2 + x2 + 0.5, i - H/2 + x1 + 0.5, - W / (2.*tan(fov/2)));    
				// direction du rayon sortant par le pixel i,j de la caméra

				u = normalized(u);

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