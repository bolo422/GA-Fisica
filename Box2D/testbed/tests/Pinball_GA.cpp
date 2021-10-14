
#include "test.h"
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <iterator>
#include <fstream>
#include <vector>
#include <algorithm> 

using namespace std;

class Pinball_GA : public Test
{
private:
	float circleRest = 0.8;
	float boxRest = 0.2;


public:
	Pinball_GA()
	{
		b2Body* ground = createBox(b2Vec2(37.2003/2 + 24.6414/2, 4.52811), b2Vec2(0.01, 0.01), 0, 0);

		criarParedes("paredes/paredes1.txt");
		criarParedes("paredes/arco direita.txt");
		criarParedes("paredes/arco direita inferior.txt");
		criarParedes("paredes/arco esquerda inferior.txt");
		criarParedes("paredes/triangulo inferior.txt");
		criarParedes("paredes/triangulo meio.txt");

		createBox(b2Vec2(12, 45), b2Vec2(0.8, 8), degToRad(40), boxRest); //rampa grande
		createBox(b2Vec2(22, 45), b2Vec2(0.4, 3), degToRad(80), boxRest); // rampa pequena

		createBox(b2Vec2(42.6342, 71.0173), b2Vec2(0.6, 3), degToRad(40), boxRest); //retangulos do topo
		createBox(b2Vec2(34.3948, 71.4005), b2Vec2(0.6, 3), degToRad(146), boxRest); //retangulos do topo
		createBox(b2Vec2(28.2392, 70.9215), b2Vec2(0.6, 3), degToRad(9), boxRest); //retangulos do topo

		createBox(b2Vec2(16.383, 65.4605), b2Vec2(4, 4), degToRad(160), boxRest * 2); //caixa grande do topo


		createCircle(b2Vec2(26.8294, 56.0204), 2, circleRest); //circulos do meio
		createCircle(b2Vec2(34.9199, 55.8419), 2, circleRest); //circulos do meio
		createCircle(b2Vec2(30.1608, 51.3208), 2, circleRest); //circulos do meio

		createCircle(b2Vec2(6.62231, 74.9578), 2, circleRest); //circulo do topo

		

		// Flippers
		{
			
			b2Vec2 p1(24.6414, 4.52811), p2(37.2003, 4.52811);
			//b2Body* ground = NULL;
			

			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			bd.position = p1;
			b2Body* leftFlipper = m_world->CreateBody(&bd);

			bd.position = p2;
			b2Body* rightFlipper = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(1.75f, 0.1f);

			b2FixtureDef fd;
			fd.shape = &box;
			fd.density = 1.0f;

			leftFlipper->CreateFixture(&fd);
			rightFlipper->CreateFixture(&fd);

			b2RevoluteJointDef jd;
			jd.bodyA = ground;
			jd.localAnchorB.SetZero();
			jd.enableMotor = true;
			jd.maxMotorTorque = 1000.0f;
			jd.enableLimit = true;

			jd.motorSpeed = 0.0f;
			jd.localAnchorA = p1;
			jd.bodyB = leftFlipper;
			jd.lowerAngle = -30.0f * b2_pi / 180.0f;
			jd.upperAngle = 5.0f * b2_pi / 180.0f;
			m_leftJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);

			jd.motorSpeed = 0.0f;
			jd.localAnchorA = p2;
			jd.bodyB = rightFlipper;
			jd.lowerAngle = -5.0f * b2_pi / 180.0f;
			jd.upperAngle = 30.0f * b2_pi / 180.0f;
			m_rightJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}

		// Circle character
		/*{
			b2BodyDef bd;
			bd.position.Set(1.0f, 15.0f);
			bd.type = b2_dynamicBody;
			bd.bullet = true;

			m_ball = m_world->CreateBody(&bd);

			b2CircleShape shape;
			shape.m_radius = 0.2f;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			m_ball->CreateFixture(&fd);
		}*/

		m_button = false;
	}

	void Step(Settings& settings) override
	{
		if (m_button)
		{
			m_leftJoint->SetMotorSpeed(20.0f);
			m_rightJoint->SetMotorSpeed(-20.0f);
		}
		else
		{
			m_leftJoint->SetMotorSpeed(-10.0f);
			m_rightJoint->SetMotorSpeed(10.0f);
		}

		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Press 'a' to control the flippers");
		m_textLine += m_textIncrement;

	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_button = true;
			break;
		}
	}

	void KeyboardUp(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_button = false;
			break;
		}
	}

	static Test* Create()
	{
		return new Pinball_GA;
	}

	b2RevoluteJoint* m_leftJoint;
	b2RevoluteJoint* m_rightJoint;
	b2Body* m_ball;
	bool m_button;

	void criarParedes(string arquivo)
	{
		ifstream is(arquivo);
		if (!is) {
			cout << "ERRO! Arquivo >>" << arquivo << "<< esta vazio ou nao pode ser carregado!\n";
			return;
		}
		istream_iterator<double> start(is), end;
		vector<double> numbers(start, end);
		cout << "Read " << numbers.size() << " " << arquivo << endl;

		// print the numbers to stdout
		cout << "numbers read in:\n";
		copy(numbers.begin(), numbers.end(),
			ostream_iterator<double>(cout, " "));
		cout << endl;


		float mult = 2.0f;
		b2Body* paredes;
		for (int i = 0; i < numbers.size() - 1; i += 2)
		{
			if (i + 3 <= numbers.size())
				paredes = createEdge(b2Vec2(numbers[i] * mult, numbers[i + 1] * mult), b2Vec2(numbers[i + 2] * mult, numbers[i + 3] * mult));
		}



	}

	b2Body* createBox(b2Vec2 pos, b2Vec2 dim, float angle, float restitution)
	{
		b2BodyDef bodyDef;

		bodyDef.type = b2_staticBody;
		bodyDef.angle = angle;
		bodyDef.position.Set(pos.x, pos.y);
		b2Body* body = m_world->CreateBody(&bodyDef);

		b2PolygonShape staticBox;
		staticBox.SetAsBox(dim.x, dim.y);
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &staticBox;
		fixtureDef.restitution = restitution;
		body->CreateFixture(&fixtureDef);
		return body;
	}


	b2Body* createCircle(b2Vec2 pos, float radius, b2Vec2 linearVelocity, float grav, float density, float friction, float restitution, bool isDynamic)
	{
		b2BodyDef bodyDef;
		bodyDef.linearVelocity.Set(linearVelocity.x, linearVelocity.y);
		bodyDef.gravityScale = grav;

		if (isDynamic)	bodyDef.type = b2_dynamicBody;
		else			bodyDef.type = b2_staticBody;
		bodyDef.position.Set(pos.x, pos.y);
		b2Body* body = m_world->CreateBody(&bodyDef);

		// Define another box shape for our dynamic body.
		b2CircleShape circle;
		circle.m_radius = radius;
		//dynamicBox.SetAsBox(dim.x, dim.y);
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &circle;
		fixtureDef.density = density;
		fixtureDef.friction = friction;
		fixtureDef.restitution = restitution;
		body->CreateFixture(&fixtureDef);

		return body;
	}

	b2Body* createCircle(b2Vec2 pos, float radius, float restitution)
	{
		b2BodyDef bodyDef;
		bodyDef.type = b2_staticBody;
		bodyDef.position.Set(pos.x, pos.y);
		b2Body* body = m_world->CreateBody(&bodyDef);

		// Define another box shape for our dynamic body.
		b2CircleShape circle;
		circle.m_radius = radius;
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &circle;
		fixtureDef.restitution = restitution;
		body->CreateFixture(&fixtureDef);

		return body;
	}

	b2Body* createEdge(b2Vec2 p1, b2Vec2 p2)
	{
		b2Body* novoObjeto;
		b2BodyDef bodyDef;
		bodyDef.type = b2_staticBody;
		b2Body* body = m_world->CreateBody(&bodyDef);

		b2EdgeShape shape;
		shape.SetTwoSided(p1, p2);
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &shape;
		body->CreateFixture(&fixtureDef);
		return body;
	}


	float degToRad(float angleDeg)
	{
		return angleDeg * b2_pi / 180.0f;
	}




};

static int testIndex = RegisterTest("Examples", "Pinball_GA", Pinball_GA::Create);
