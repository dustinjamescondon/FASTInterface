#include "TimePlot.h"

TimePlot::TimePlot(int x, int y, int w, int h, double minVal, double maxVal)
{
	mainTexture.create(w, h);
	mainTexture.clear(sf::Color(255, 255, 255));
	this->minVal = minVal;
	this->maxVal = maxVal;
	width = (float)w;
	height = (float)h;
	xpos = x;
	ypos = y;
}

void TimePlot::plot(double time, double value)
{
	sf::Texture tex(mainTexture.getTexture());
	scrollSprite = sf::Sprite(tex);
	scrollSprite.move(-1.0, 0.0f);

	mainTexture.clear(sf::Color::White);
	mainTexture.draw(scrollSprite);
	mainTexture.display();

	sf::CircleShape point(2.0);
	point.setFillColor(sf::Color(255, 0, 0));
	double fracOfHeight = value / maxVal;
	point.setPosition(sf::Vector2f(width - 2, height - 2 - (fracOfHeight * height)));
	
	mainTexture.draw(point);
	mainTexture.display();
}

void TimePlot::draw(sf::RenderWindow& wnd)
{
	scrollSprite = sf::Sprite(mainTexture.getTexture());
	scrollSprite.setPosition(xpos, ypos);
	wnd.draw(scrollSprite);
}


const sf::Texture& TimePlot::getTexture() const
{
	return mainTexture.getTexture();
}
