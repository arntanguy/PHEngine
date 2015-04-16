/******************************************************************************
     Copyright (C) 2012-2013  TANGUY Arnaud arn.tanguy@gmail.com
*                                                                             *
* This program is free software; you can redistribute it and/or modify        *
* it under the terms of the GNU General Public License as published by        *
* the Free Software Foundation; either version 2 of the License, or           *
* (at your option) any later version.                                         *
*                                                                             *
* This program is distributed in the hope that it will be useful,             *
* but WITHOUT ANY WARRANTY; without even the implied warranty of              *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                *
* GNU General Public License for more details.                                *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc.,     *
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.                 *
 ******************************************************************************/

/**
 * This code is inspired by the particle generator tutorial which can be found here (French)
 * http://www.siteduzero.com/tutoriel-3-35438-programmer-un-generateur-de-particules.html
 **/

#include "ParticleGenerator.h"
#include "CGEngine/PlaneEntity.h"
#include <glm/gtx/norm.hpp>

#define THRESHOLD_DISTANCE 0.2

double myRand(double min, double max)
{
    return (double) (min + ((float) rand() / RAND_MAX * (max - min + 1.0)));
}

ParticleGenerator::ParticleGenerator(glm::vec3 emitterPos, int maxParticles) {
    mEmitterPosition = emitterPos;
    mMaxParticles = maxParticles;
    mParticles = new Particle[maxParticles];

    mA = 0.6;
    mG = 0.01;
    mBounce = 1;

    initParticles();
}

ParticleGenerator::~ParticleGenerator() {
    delete mParticles;
}

void ParticleGenerator::initParticles()
{
    for(int i=0; i<mMaxParticles; i++)   // Boucle sur toutes les particules
    {
        initParticle(i);
    }
}

void ParticleGenerator::initParticle(int i)
{
        mParticles[i].active = true;       // On rend la particule active
        mParticles[i].life = 1.0;   // Maximum de vie

        mParticles[i].fade = myRand(0.001,0.002);   // Vitesse de disparition aléatoire
        mParticles[i].color.x=myRand(0.0,1.0);  // Quantité aléatoire de Rouge
        mParticles[i].color.y=myRand(0.0,1.0);  // Quantité aléatoire de Vert
        mParticles[i].color.z=myRand(0.0,1.0);  // Quantité aléatoire de Bleu

        mParticles[i].position.x = mEmitterPosition.x;
        mParticles[i].position.y = mEmitterPosition.y;
        mParticles[i].position.z = mEmitterPosition.z;

        //mParticles[i].velocity.x = myRand(-0.001,0.001);   // Vitesse aléatoire
        //mParticles[i].velocity.y = myRand(0.001,0.001);
        //mParticles[i].velocity.z = myRand(-0.001,0.001);
        mParticles[i].velocity.x = myRand(-0.5,0.5);   // Vitesse aléatoire
        mParticles[i].velocity.y = myRand(0.1,0.8);
        mParticles[i].velocity.z = myRand(-0.01,0.01);

        mParticles[i].force.x = 0;
        mParticles[i].force.y = 0;
        mParticles[i].force.z = 0;

        mParticles[i].mass = myRand(0.1,0.5);

        mParticles[i].size = 0.5;
}


// This routine returns up to 3 camera directions: which way is "right", "up" and which way is the camera pointing ("look")
// in OpenGL coordinates.  In other words, this is which way the user's SCREEN is pointing in OpenGL "local" coordinates.
// (If the user is facing true north at the origin and is not rolled, these functiosn would be trivially easy because
// right would be 1,0,0, up would be 0,1,0 and look would be 0,0,1.  (NOTE: the look vector points TO the user, not
// FROM the user.)
//
// To draw a billboard centered at C, you would use these coordinates:
//
// c-rgt+up---c+rgt+up
// |                 |
// |        C        |
// c-rgt-up---c+rgt-up
//
static void camera_directions(
						float * out_rgt,		// Any can be NULL
						float * out_up ,
						float * out_look)
{
	float m[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, m);

	// Roughly speaking, a modelview matrix is made up more or less like this:
	// [ EyeX_x EyeX_y EyeX_z    a
	//   EyeY_x EyeY_y EyeY_z    b
	//   EyeZ_x EyeZ_y EyeZ_z    c
	//   um don't look down here ]
	// where a, b, c are translations in _eye_ space.  (For this reason, a,b,c is not
	// the camera location - sorry!)

	if(out_rgt) {
		out_rgt[0] = m[0];
		out_rgt[1] = m[4];
		out_rgt[2] = m[8];
	}
	if(out_up) {
		out_up[0] = m[1];
		out_up[1] = m[5];
		out_up[2] = m[9];
	}
	if(out_look) {
		out_up[0] = m[2];
		out_up[1] = m[6];
		out_up[2] = m[10];
	}
}

void ParticleGenerator::render(float ellapsedtime)
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    // Right and up direction of camera
    float r[3], u[3];
	camera_directions(r,u,NULL);
    //std::cout << "CAM: " << r[0] << " " << r[1] << " " << r[2] << std::endl;
    //std::cout << "CAM UP: " << u[0] << " " << u[1] << " " << u[2] << std::endl;

    glPushMatrix();

        int a = 0;
        for(int i=0; i<mMaxParticles; i++) // For each particle
        {
            if(mParticles[i].active)         // If it is active
            {
                float x = mParticles[i].position.x;   // Get its position
                float y = mParticles[i].position.y;
                float z = mParticles[i].position.z;
                //std::cout << "pos " << x << "," << y << "," << z << std::endl;

                /* Particle color, transparence is life */
                glColor4d(mParticles[i].color.r, mParticles[i].color.g, mParticles[i].color.b, mParticles[i].life);

                float size = mParticles[i].size;
                //glBegin(GL_TRIANGLES);
                //    glVertex3f(x, y, z);
                //    glVertex3f(x+size, y, z);
                //    glVertex3f(x+size, y+size, z);

                //    glVertex3f(x+size, y+size, z);
                //    glVertex3f(x, y+size, z);
                //    glVertex3f(x, y, z);
                //glEnd();

                /*
CAM: 1 0 0
CAM UP: 0 1 0
glVertex3f(x*size, size, size
 */
                glBegin(GL_TRIANGLE_STRIP);
                    //float size = mParticles[i].size;
                    glVertex3f(x+size*(r[0]+u[0]), y+size*(r[1]+u[1]),z+size*(r[2]+u[2]));
                    glVertex3f(x+size*(-r[0]+u[0]),y+size*(-r[1]+u[1]),z+size*(-r[2]+u[2]));
                    glVertex3f(x+size*(r[0]-u[0]), y+size*(r[1]-u[1]),z+size*(r[2]-u[2]));
                    glVertex3f(x+size*(-r[0]-u[0]),y+size*(-r[1]-u[1]),z+size*(-r[2]-u[2]));
                glEnd();


                applyForces(i, ellapsedtime);
                handleCollisions(i);


                mParticles[i].life -= mParticles[i].fade;

                /* If particle died, we regenerate it */
                if (mParticles[i].life <= 0.0)
                {
                    initParticle(i);
                }
            }
        }
    glPopMatrix();
    glPopAttrib();
}

void ParticleGenerator::applyForces(int i, float timestep)
{
    mParticles[i].force = glm::vec3(0,0,0);
    /**
     * Apply Gravity
     * fgrav = -m*G
     **/
    mParticles[i].force.y += -mG * mParticles[i].mass;

    /**
     * Apply archimede
     * Pa = -p*V*g
     **/
    float volume = mParticles[i].size*mParticles[i].size;
    mParticles[i].force.y += mA*volume*mG;

    /***
     * Compute acceleration, speed, and position
     */
    glm::vec3 acceleration = (1.f/mParticles[i].mass) * mParticles[i].force;
    mParticles[i].velocity += acceleration * timestep;
    mParticles[i].position += mParticles[i].velocity*timestep;

    /* Apply forces */
    //std::vector<glm::vec3>::const_iterator it;
    //for(it = mForces.begin(); it != mForces.end(); it++) {
    //    mParticles[i].x += (*it).x;
    //    mParticles[i].y += (*it).y;
    //    mParticles[i].z += (*it).z;
    //}
}

void ParticleGenerator::handleCollisions(int i)
{
    Particle particle = mParticles[i];
    std::vector<PlaneEntity*>::const_iterator it;
    for(it = mCollisionPlanes.begin(); it != mCollisionPlanes.end(); it++) {
        glm::vec3 normal = (*it)->getNormal();
        glm::vec3 center = (*it)->getCenter();

        glm::vec3 planeToParticle = particle.position - center;
        float dotProduct = glm::dot(normal, planeToParticle);

        // If there is a collision
        if(dotProduct < THRESHOLD_DISTANCE) {
            //std::cout << "Intersection at " << particle.position.x << "," << particle.position.y << "," << particle.position.z << std::endl;
            //std::cout << "Velocity: " << particle.velocity.x << "," << particle.velocity.y << "," << particle.velocity.z << std::endl;

            // If particle is heading deaper
            if(glm::dot(normal, particle.velocity) < THRESHOLD_DISTANCE) {
                //std::cout << "Particle is heading deeper into the plane" << std::endl;
                // Bouce back particle
                // Velocity in normal direction scaled by -kr (reflection)
                // Velocity in tangential direction preserved
                glm::vec3 normalVelocity = glm::dot(particle.velocity, normal)*normal;
                glm::vec3 tangentialVelocity = particle.velocity-glm::dot(particle.velocity, normal)*normal;
                //std::cout << "tangential: " << tangentialVelocity.x << "," << tangentialVelocity.y << "," << tangentialVelocity.z << std::endl;

                glm::vec3 newVelocity = tangentialVelocity - mBounce * normalVelocity;
                mParticles[i].velocity = newVelocity;


                /**
                 * Post-processing to correct the position
                 **/
                // Project vector onto normal to get minimal distance from point to plane
                float distance = glm::abs(glm::dot(normal, planeToParticle));
                // Move particle back to the right side of the plane at the correct coordinates
                mParticles[i].position += distance*normal;
            }
        }
    }
}


void ParticleGenerator::changeSize(double size) {
    for(int i=0; i<mMaxParticles; i++) {
        mParticles[i].size = size;
    }
}

void ParticleGenerator::addForce(glm::vec3 f)
{
    mForces.push_back(f);
}


void ParticleGenerator::addCollisionPlane(PlaneEntity *plane)
{
    mCollisionPlanes.push_back(plane);
}

