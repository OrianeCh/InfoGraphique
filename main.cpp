#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

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
double dot(const Vector& a, const Vector& b) 
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


class Ray {
public:
	explicit Ray(const Vector& C, const Vector& u) : C(C), u(u) {
	};
	Vector C, u;
};

class Sphere {
public:
	explicit Sphere(const Vector& O, double R, const Vector& albedo) : O(O), R(R), albedo(albedo) {
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
private :
	Vector O;
	double R;
};

class Scene {
public:
	Scene() {};
	bool intersect(const Ray& r, Vector& P, Vector& N, Vector& albedo, double &t)
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
			}
		}
		return has_inter;

	}
	std::vector<Sphere> objects;

};

int main() {
	int W = 512;
	int H = 512;

	Vector C(0, 0, 55);    // centre de la caméra
	Scene scene;
	Sphere S1(Vector(0, 0, 0), 10, Vector(1, 1, 1));
	Sphere Ssol(Vector(0, -1000, 0), 990, Vector(1, 1, 1));
	Sphere Smur1(Vector(-1000, 0, 0), 940, Vector(1, 0., 0.));
	Sphere Smur2(Vector(1000, 0, 0), 940, Vector(0., 1, 0.));
	Sphere Smur3(Vector(0, 0, -1000), 940, Vector(0., 0., 1));
	Sphere Smur4(Vector(0, 0, 1000), 940, Vector(0., 1, 1));
	Sphere Splafond(Vector(0, 1000, 0), 940, Vector(1, 0., 0.));
	scene.objects.push_back(S1);
	scene.objects.push_back(Ssol);
	scene.objects.push_back(Smur1);
	scene.objects.push_back(Smur2);
	scene.objects.push_back(Smur3);
	scene.objects.push_back(Smur4);
	scene.objects.push_back(Splafond);

	double fov = 60 * M_PI /180;
	double I = 5E9;    // Intensité de la source lumineuse

	Vector L(-10, 20, 40);    // Position de la source lumineuse

	std::vector<unsigned char> image(W*H * 3, 0);
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
			Vector P, N, albedo;
			double t;
			bool inter = scene.intersect(r, P, N, albedo, t);
			Vector color(0, 0, 0);    // noir 
			if (inter) 
			{
				Vector PL = L-P;
				double d = sqrt(PL.carreNorm());    
				// distance entre le point d'intersection et la lumière
				Vector shadowP, shadowN, shadowAlbedo;
				double shadowt;
				Ray shadowRay(P + 0.001*N, PL/d);
				bool ombre = scene.intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowt);
				if (ombre && shadowt < d) {
					color = Vector(0., 0., 0.);
				} else {
					color = I / (4*M_PI*d*d) * albedo/M_PI *std::max(0., dot(N, PL/d));
				}
			};
			image[((H-i-1)*W + j) * 3 + 0] = std::min(255., pow(color[0], 0.45));
			image[((H-i-1)*W + j) * 3 + 1] = std::min(255., pow(color[1], 0.45));
			image[((H-i-1)*W + j) * 3 + 2] = std::min(255., pow(color[2], 0.45));
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}