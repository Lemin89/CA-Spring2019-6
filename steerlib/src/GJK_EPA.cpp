#include <vector>
#include <algorithm>
#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	if (_shapeA.size() < 3 || _shapeB.size() < 3)
        return false;
    /*
	 *	GJK
	 */
    float tmp, min;
    bool collision, abool, bbool;
	float a, b, c, k1, k2;
    Util::Vector v0(0, 0, 0);                   // direction vector
	Util::Vector zero(0, 0, 0);                 // shorthand for (0, 0, 0)
    Util::Vector minVec(0, 0, 0);               // we use this to store the minimum/maximum vector/point along v0
    std::vector<Util::Vector> simplex;          // the simplex/polygon created by gjk and then epa
	std::vector<Util::Vector> minkowskiDiff;    // Minkowski difference
    collision = abool = bbool = false;
    std::cout << "A: ";
	for (unsigned i = 0; i < _shapeA.size(); ++i) {
//		std::cout << _shapeA[i];
		for (unsigned j = 0; j < _shapeB.size(); ++j) {
            minkowskiDiff.push_back(_shapeA[i] - _shapeB[j]);
		}
	}
	std::cout << std::endl << "B: ";
/*	for (unsigned i = 0; i < _shapeB.size(); ++i)
		std::cout << _shapeB[i];
	std::cout << std::endl;  */
    v0 = minkowskiDiff[0];
    while (true) {
        /*
            Note: points and vector are used interchangably. Therefore I'm using vectors to represent points as well.
        */
        // Base case simplex has 0 or 1 elements.
        // We take the first point in the minkowskiDifference 6 lines above
        // and look for the smallest dot product
		if (simplex.size() < 2){
			min = Util::dot(v0, minkowskiDiff[0]);
	        for (unsigned i = 1; i < minkowskiDiff.size(); ++i) {
	            tmp = Util::dot(v0, minkowskiDiff[i]);
	            if (tmp < min){
	                min = tmp;
	                minVec = minkowskiDiff[i];
	            }
	        }
	        if (minVec == v0){
			   // converging without a collision
			   collision = false;
			   break;
	        }
            // set v0 = the point with the smallest dot product 
            // this is used again when simplex.size == 1
	        v0 = minVec;
	        simplex.push_back(Util::Vector(v0.x, 0, v0.z));
		}
        // After we have two points in the simplex, we look for a third point and
        // search for the smallest dot product along that point
		else if (simplex.size() == 2){
            // check to see if we need an artificial v0
            if (std::abs(simplex[0].z - simplex[1].z) < 0.0001) {
                // horizontal line if deltaX != 0
                abool = true;
                v0.x = 0;
                v0.y = 0;
                v0.z = 1;
A:
                v0.z = -1;
                abool = false;
            }
            else if (std::abs(simplex[0].x - simplex[1].x) < 0.0001) {
                // vertical line if deltaY != 0
                bbool = true;
                v0.x = 1;
                v0.y = 0;
                v0.z = 0;
B:
                v0.x = -1;
                bbool = false;
            }
            else {
    			k1 = (simplex[0].z - simplex[1].z) / (simplex[0].x - simplex[1].x);
    			k2 = -(simplex[0].x - simplex[1].x) / (simplex[0].z - simplex[1].z);
    			v0.x = (simplex[0].z - k1 * simplex[0].x) / (k2 - k1);
    			v0.y = 0;
    			v0.z = k2 * v0.x;
            }
			min = Util::dot(v0, minkowskiDiff[0]);
			for (unsigned i = 1; i < minkowskiDiff.size(); ++i){
				tmp = Util::dot(v0, minkowskiDiff[i]);
				if (tmp < min) {
					min = tmp;
					minVec = minkowskiDiff[i];
				}
			}
			// Check if v0 is almost equal to minVec and if simplex already contains minVec
			a = std::abs(v0.x - minVec.x);
			b = std::abs(v0.z - minVec.z);
			if ((a < 0.0001 && b < 0.0001) || simplex[0] == minVec || simplex[1] == minVec) {
                if (abool)
                    goto A;
                if (bbool)
                    goto B;
				collision = false;
				break;
			}
			simplex.push_back(Util::Vector(minVec.x, 0, minVec.z));
            // Now we have three points in the simplex and we check if the simplex contains 0
            if (pointTriangle(simplex, zero)){
				collision = true;
				break;
			}
			else {
                abool = bbool = false;
			    // remove the furthest point from the origin
				a = Util::dot(simplex[0], simplex[0]);
				b = Util::dot(simplex[1], simplex[1]);
				c = Util::dot(simplex[2], simplex[2]);
				if (a >= b) {
					if (a >= c)
						simplex.erase(simplex.begin());
					else
						simplex.erase(simplex.begin() + 2);
				}
				else if (b >= c) {
					simplex.erase(simplex.begin() + 1);
				}
				else
				   simplex.erase(simplex.begin() + 2);
			}
            abool = bbool = false;
		}
    }
	/*
	 *	EPA
	 */
    if (collision) {
/*        std::cout << "Simplex: ";
        for (unsigned i = 0; i < simplex.size(); ++i)
            std::cout << simplex[i];
        std::cout << std::endl; */
        Util::Vector AB;
        volatile int j, minI, minJ; 
        while (1) {
            // The next two lines use the dot product and vector subtraction to get the normal of the
            // edge defined by the first two points in the simplex.
            AB = simplex[1] - simplex[0];
            minVec = simplex[0] + Util::dot(-simplex[0], AB) * AB  / Util::dot(AB, AB);
            minVec.y = 0;
            min = minVec.lengthSquared();
            minI = 0, minJ = 1;
//            std::cout << "minVec: " << minVec << std::endl;
            for (unsigned i = 1; i < simplex.size(); ++i){
                /*
                 *	Test points i=A and j=B
                 *	c = projection of -A onto AB
                 *	(A - c) is the vector we want (perpendicular to the polygon edge)
                 */
                if (i == simplex.size() - 1)
                    j = 0;          // loop around if the last element
                else
                    j = i+1;
                // calculate the normal to the edge again
                AB = simplex[j] - simplex[i];
                v0  = simplex[i] + Util::dot(-simplex[i], AB) * AB / Util::dot(AB, AB);
//               std::cout << "V0: " << v0 << std::endl;
                if (v0.x == 0 && v0.y == 0 && v0.z == 0){
                    //std::cout << "0 vector homie, t'was not expected" << std::endl;
                }
                v0.y = 0;
                tmp = v0.lengthSquared();
                // Compare this normal to the last one and store the smaller one
                // Also store the index of this normal in case we need to insert this point into the polygon.
                if (tmp < min) {
                    minVec.x = v0.x;
                    minVec.z = v0.z;
                    min = tmp;
                    minI = i;
                    minJ = j;
                }
            }
            /*
             * Find the furthest point along vector (A - c)
             *** The use of min is reversed in this case, no need for another float and vector
             */
             v0.x = minVec.x;
             v0.z = minVec.z;
             min = Util::dot(v0, minkowskiDiff[0]);
             minVec = minkowskiDiff[0];
//             std::cout << "Edge v0:" << v0 << ". Edge minvec: " << minVec << std::endl;
             for (unsigned i = 1; i < minkowskiDiff.size(); ++i) {
                tmp = Util::dot(v0, minkowskiDiff[i]);
                if (tmp > min) {
//                    std::cout << "VZero: " << v0 << std::endl << "MinVec: " << minVec << std::endl;
                    min = tmp;
                    minVec = minkowskiDiff[i];
                }
             }
            /*
             * If the new point's direction and magnitute did not change much compared to (A - c) we can finish
             * otherwise we insert the new point between A and B
             */
//             std::cout << "Edge v0:" << v0 << ". Edge minvec: " << minVec << std::endl;
             if ((v0 - minVec).lengthSquared() < 0.1) {
                return_penetration_depth = minVec.length();
                return_penetration_vector = minVec / return_penetration_depth;
                break;
            }
            else if (minVec == simplex[minI] || minVec == simplex[minJ]) {
                // If the simplex already contains this point
                // if the minVec is one of the points that builds the edge
                a = simplex[minI].length();
                b = simplex[minJ].length();
                c = v0.length();
				if (a <= b) {
					if (a <= c) {
                        return_penetration_depth = a;
                        return_penetration_vector = simplex[minI] / a;
                    }
					else {
                        return_penetration_depth = c;
                        return_penetration_vector = v0 / c;
                    }
				}
				else if (b <= c) {
                    return_penetration_depth = b;
                    return_penetration_vector = simplex[minJ] / b;
				}
				else {
                    return_penetration_depth = c;
                    return_penetration_vector = v0 / c;
                }
                break;
            }
            simplex.insert(simplex.begin() + minJ, Util::Vector(minVec.x, 0, minVec.z));
        }
    }
	return collision;
}

// Check if a triangle contains a point
bool SteerLib::GJK_EPA::pointTriangle(std::vector<Util::Vector> triangle, Util::Vector point){
	bool ret = false;
	if (triangle.size() != 3)
		return false;
	Util::Vector v0 = triangle[0] - triangle[2];
	Util::Vector v1 = triangle[1] - triangle[2];
	Util::Vector v2 = point - triangle[2];
	float d00 = Util::dot(v0, v0);
	float d01 = Util::dot(v0, v1);
	float d11 = Util::dot(v1, v1);
	float d20 =	Util::dot(v2, v0);
	float d21 = Util::dot(v2, v1);
	float denom = (d00 * d11) - (d01 * d01);

	float a = (d11 * d20 - d01 * d21) / denom;
	float b = (d00 * d21 - d01 * d20) / denom;
	float c = 1 - a - b;
	if (a >= 0 && a <= 1 && b >= 0 && b <= 1 && c >= 0 && c <= 1)
		ret = true;
	else {
		float e = 0.0001;
		if (a >= -e && a <= 1+e && b >= -e && b <= 1+e && c >= -e && c <= 1+e)
		   ret = true;
	}
//	std::cout << "Triangle: " << triangle[0] << ", " << triangle[1] << ", " << triangle[2] << ". point: " << point << ". true: " << ret << std::endl;
	return ret;
}

