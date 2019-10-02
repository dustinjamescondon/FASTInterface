#pragma once

#include <SFML\Graphics.hpp>

class TimePlot
{
public:
	TimePlot(int x, int y, int w, int h, double min, double max);

	void plot(double time, double val);

	void draw(sf::RenderWindow&);

	const sf::Texture& getTexture() const;

private:
	sf::RenderTexture mainTexture;
	sf::Sprite scrollSprite;
	double minVal, maxVal;
	float width, height;
	int xpos, ypos;
};

