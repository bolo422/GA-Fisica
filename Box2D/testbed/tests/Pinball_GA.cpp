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
#include "imgui/imgui.h"

using namespace std;

b2Vec2 decomposeVector(float angleDeg, float magnitude);
float degToRad(float angleDeg);


class Pinball_GA : public Test //você cria a sua classe derivada da classe base Test
{

private:
	
	b2RevoluteJoint* rjoint;
	b2RevoluteJoint* rjoint2;
	b2Body* bola;
	float bolaGrav = 1.0f;
	bool bolaImpulse = true;
	bool forcaManual = false;
	float forcaMotorFlippers = 100.0f;
	bool a_button;
	bool d_button;
	float flipperForce = 10000.0f;
	int x = 57, y = 1;
	bool a = false;
	bool bolacri = false;
	bool aumentando = true;
	float circleRest = 0.8;
	float topcirclerest = 0.5;
	float boxRest = 1.2;
	int pontos = 0;
	int melhorPontuacao = 0;
	float globalForce = 0, globalAngle = 90;
	float forca = 0;
	

public:
	Pinball_GA() {
		carregarRest();
		melhorPontuacao = carregarPontos();
		criarParedes("paredes/paredes1.txt");
		criarParedes("paredes/arco direita.txt");
		criarParedes("paredes/arco direita inferior.txt");
		criarParedes("paredes/arco esquerda inferior.txt");
		b2Body* anchorEsquerda = createBox(b2Vec2(20.8089, 7.31567), b2Vec2(5, 0.4), degToRad(-37), 0);
		b2Body* anchorDireita = createBox(b2Vec2(41.1089, 7.31567), b2Vec2(5, 0.4), degToRad(37), 0);
		

		criarParedes("paredes/triangulo inferior.txt");
		criarParedes("paredes/triangulo meio.txt");

		createBox(b2Vec2(12, 45), b2Vec2(0.8, 8), degToRad(40), boxRest); //rampa grande
		createBox(b2Vec2(22, 45), b2Vec2(0.4, 3), degToRad(80), boxRest); // rampa pequena

		createBox(b2Vec2(42.6342, 71.0173), b2Vec2(0.6, 3), degToRad(40), boxRest); //retangulos do topo
		createBox(b2Vec2(34.3948, 71.4005), b2Vec2(0.6, 3), degToRad(146), boxRest); //retangulos do topo
		createBox(b2Vec2(28.2392, 70.9215), b2Vec2(0.6, 3), degToRad(9), boxRest); //retangulos do topo

		createBox(b2Vec2(16.383, 65.4605), b2Vec2(4, 4), degToRad(160), boxRest*2); //caixa grande do topo


		createCircle(b2Vec2(26.8294, 56.0204), 2, circleRest); //circulos do meio
		createCircle(b2Vec2(34.9199, 55.8419), 2, circleRest); //circulos do meio
		createCircle(b2Vec2(30.1608, 51.3208), 2, circleRest); //circulos do meio

		createCircle(b2Vec2(6.62231, 74.9578), 2, topcirclerest); //circulo do topo

		// Flippers
		

		b2Vec2 pEsquerda(24.6414, 4.52811), pDireita(37.2003, 4.52811);
		float flipperSize = 5.4;
		b2Body* fEsquerda = createBox(b2Vec2(pEsquerda.x, pEsquerda.y), b2Vec2(flipperSize, 0.3), b2Vec2(0, 0), 0, 1, 0, 0, true);
		b2Body* fDireita = createBox(b2Vec2(pDireita.x, pDireita.y), b2Vec2(flipperSize, 0.3), b2Vec2(0, 0), 0, 1, 0, 0.4, true);


		b2Vec2 worldAnchorOnBody1 = fEsquerda->GetWorldPoint(b2Vec2(-flipperSize , 0));
		b2RevoluteJointDef rJointDef;
		rJointDef.Initialize(fEsquerda, anchorEsquerda, pEsquerda);
		rJointDef.lowerAngle = degToRad(-45); // -45 degrees
		rJointDef.upperAngle = degToRad(35); // 45 degrees
		rJointDef.enableLimit = true;
		rJointDef.enableMotor = true;
		rJointDef.maxMotorTorque = flipperForce;
		rjoint = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef);

		b2Vec2 worldAnchorOnBody2 = fDireita->GetWorldPoint(b2Vec2(flipperSize, 0));
		b2RevoluteJointDef rJointDef2;
		rJointDef2.Initialize(fDireita, anchorDireita, pDireita);
		rJointDef2.lowerAngle = degToRad(-35); // -45 degrees
		rJointDef2.upperAngle = degToRad(45); // 45 degrees
		rJointDef2.enableLimit = true;
		rJointDef2.enableMotor = true;
		rJointDef2.maxMotorTorque = flipperForce;
		rjoint2 = (b2RevoluteJoint*)m_world->CreateJoint(&rJointDef2);

		

	}

	void BeginContact(b2Contact* contact)
	{

		for (b2Contact* c = m_world->GetContactList(); c; c = c->GetNext())
		{
			if (c->GetRestitution() == circleRest && a == false)
			{
				pontos += 10;
			}
			if (c->GetRestitution() == boxRest && a == false)
			{
				pontos += 5;
			}
			if (c->GetRestitution() == topcirclerest && a == false)
			{
				pontos += 30;
			}
		}
		a = true;
	}
	void EndContact(b2Contact* contact)
	{
		a = false;


	}

	void Step(Settings& settings) override
	{

		if (!a_button){
		
			rjoint->SetMotorSpeed(forcaMotorFlippers);
		}
		else
		{
			rjoint->SetMotorSpeed(forcaMotorFlippers * -1 / 2);
		}

		if (!d_button)
		{
			rjoint2->SetMotorSpeed(forcaMotorFlippers * -1);
		}
		else
		{
			rjoint2->SetMotorSpeed(forcaMotorFlippers / 2);
		}



		if (aumentando && !forcaManual) {
			forca++;
			if (forca > 150) {
				aumentando = false;
			}
		}
		if (!aumentando && !forcaManual) {
			forca--;
			if (forca < 20) {
				aumentando = true;
			}
		}
		
		if (bolacri) {
			if (bola->GetPosition().y < -16.4553) {
				bola->SetEnabled(false);
				bolacri = false;
				if (pontos > melhorPontuacao) {
					melhorPontuacao = pontos;
					salvarPontos(pontos);
					melhorPontuacao = carregarPontos();
				}
				pontos = 0;
			}
			if (bola->GetPosition().y < 0.8 && bola->GetPosition().x > 54.523) {
				bolaImpulse = true;
			}
		}

		//Chama o passo da simulação e o algoritmo de rendering
		Test::Step(settings);

		//show some text in the main screen
		g_debugDraw.DrawString(5, m_textLine, "Pinball GA - Erick e Jonathan");
		m_textLine += 15;

	}

	void UpdateUI() override
	{

		
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(210.0f, 285.0f));
		ImGui::Begin("PINBALL - Erick e Jonathan", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::TextColored(ImVec4(0, 1, 1, 1), "Instrucoes:\nA e D para usar as palhetas\nS para lancar a bolinha");

		if (ImGui::Button("FORCA"))
		{
			Create();
		}
		if(!forcaManual)
			ImGui::SliderFloat("", &forca, 20.0f, 150.0f, "%.0f");
		else
			ImGui::SliderFloat("", &forca, 20.0f, 500.0f, "%.0f");

		

		ImGui::Checkbox("Forca Manual", &forcaManual);


		float showGrav = bolaGrav*10;
		ImGui::SliderFloat("Gravidade: ", &showGrav, 10.0f, 50.0f, "%.0f");
		bolaGrav = showGrav / 10;
		ImGui::SliderFloat("Forca Flippers: ", &forcaMotorFlippers, 0.0f, 100.0f, "%.0f");
		ImGui::Text("Pontos:%d", pontos);
		ImGui::Text("Melhor Pontuacao:%d", melhorPontuacao);

		ImGui::End();

		
	}

	static Test* Create()  //a classe Test que instancia um objeto da sua nova classe
						   //o autor da Box2D usa um padrão de projeto chamado Factory
						   //para sua arquitetura de classes
	{
		return new Pinball_GA;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			a_button = true;
			break;
		case GLFW_KEY_D:
			d_button = true;
			break;

		case GLFW_KEY_S:
			if (key == GLFW_KEY_S) {
				
				globalForce = forca;
			}
			break;

		
		}
	}

	void KeyboardUp(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			a_button = false;
			break;
		case GLFW_KEY_D:
			d_button = false;
			break;
		case GLFW_KEY_S:
			if (key == GLFW_KEY_S  ) {
			
				if (bolacri == false)
				{
					b2BodyDef bodyDef;
					bodyDef.type = b2_dynamicBody;
					bodyDef.position.Set(x, y);
					bodyDef.gravityScale = bolaGrav;
					bola = m_world->CreateBody(&bodyDef);

					// Define another box shape for our dynamic body.
					b2CircleShape circle;
					circle.m_radius = 0.7;
					//dynamicBox.SetAsBox(dim.x, dim.y);
					b2FixtureDef fixtureDef;
					fixtureDef.shape = &circle;
					fixtureDef.density = 1;
					fixtureDef.friction = 1;
					fixtureDef.restitution = 0.2;
					bola->CreateFixture(&fixtureDef);

					b2Vec2 force = decomposeVector(globalAngle, globalForce);
					bola->ApplyLinearImpulse(force, bola->GetWorldCenter(), true);
					bolacri = true;
					bolaImpulse = false;
				}
				else if (bolaImpulse) {
					b2Vec2 force = decomposeVector(globalAngle, globalForce);
					bola->ApplyLinearImpulse(force, bola->GetWorldCenter(), true);
					bolaImpulse = false;
				}
				
			}
			break;
		}
	}

	b2Body* createBox(b2Vec2 pos, b2Vec2 dim, b2Vec2 linearVelocity, float grav, float density, float friction, float restitution, bool isDynamic)
	{
		b2BodyDef bodyDef;
		bodyDef.linearVelocity.Set(linearVelocity.x, linearVelocity.y);
		bodyDef.gravityScale = grav;

		if (isDynamic)	bodyDef.type = b2_dynamicBody;
		else			bodyDef.type = b2_staticBody;

		bodyDef.position.Set(pos.x, pos.y);
		b2Body* body = m_world->CreateBody(&bodyDef);

		b2PolygonShape dynamicBox;
		dynamicBox.SetAsBox(dim.x, dim.y);
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &dynamicBox;
		fixtureDef.density = density;
		fixtureDef.friction = friction;
		fixtureDef.restitution = restitution;
		body->CreateFixture(&fixtureDef);
		return body;
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


	b2Body* criarParedes(string arquivo)
	{
		ifstream is(arquivo);
		if (!is) {
			cout << "ERRO! Arquivo >>" << arquivo << "<< esta vazio ou nao pode ser carregado!\n";
		}
		istream_iterator<double> start(is), end;
		vector<double> numbers(start, end);
		//cout << "Read " << numbers.size() << " " << arquivo << endl;

		// print the numbers to stdout
		//cout << "numbers read in:\n";
		//copy(numbers.begin(), numbers.end(),
		//	ostream_iterator<double>(cout, " "));
		//cout << endl;


		float mult = 2.0f;
		b2Body* paredes;
		for (int i = 0; i < numbers.size()-1; i += 2)
		{
			if(i+3<=numbers.size())
				paredes = createEdge(b2Vec2(numbers[i] * mult, numbers[i + 1] * mult), b2Vec2(numbers[i + 2] * mult, numbers[i + 3] * mult));
		}
		is.close();
		return paredes;
	}

	void salvarPontos(int pontos) {
		ofstream is("melhorPontuacao.txt");
		if (!is) {
			cout << "ERRO! Arquivo >> melhorPontuacao.txt << esta vazio ou nao pode ser carregado!\n";
		}
		is << to_string(pontos);
		is.close();
	}

	int carregarPontos() {
		ifstream is("melhorPontuacao.txt");
		if (!is) {
			cout << "ERRO! Arquivo >> melhorPontuacao.txt << esta vazio ou nao pode ser carregado!\n";
		}
		string n;
		getline(is, n);
		is.close();
		return stoi(n);
	}

	void carregarRest() {
		ifstream is("configs.txt");
		if (!is) {
			cout << "ERRO! Arquivo >> configs.txt << esta vazio ou nao pode ser carregado!\n";
		}
		string n;
		int i = 0;

		/*float circleRest = 0.8;
		float topcirclerest = 0.5;
		float boxRest = 1.2;*/

		while (is.good()) {
			getline(is, n);
			if (n[0] != '#') {
				switch (i)
				{
				case 0: boxRest = stof(n) + 0.0008; i++;	break;
				case 1: circleRest = stof(n) + 0.0007; i++;	break;
				case 2: topcirclerest = stof(n) + 0.0009; i++;	break;
				}
			}
		}
			if (boxRest < 0.5)
				boxRest = 0.5;

			if (topcirclerest < 0.5)
				topcirclerest = 0.5;

			if (circleRest < 0.5)
				circleRest = 0.5;

						//DEBUG
			//cout << "\nboxRest: " << boxRest << "\circleRest: " << circleRest << "\topcirclerest: " << topcirclerest << endl;
		
		is.close();
	}

};

//Aqui fazemos o registro do novo teste 
static int testIndex = RegisterTest("Examples", "Pinball_GA", Pinball_GA::Create);


b2Vec2 decomposeVector(float angleDeg, float magnitude) 
{
	b2Vec2 decomposed;

	decomposed.x = magnitude * cos(degToRad(angleDeg));
	decomposed.y = magnitude * sin(degToRad(angleDeg));
	return decomposed;
}


float degToRad(float angleDeg)
{
	return angleDeg * b2_pi / 180.0f;
}



