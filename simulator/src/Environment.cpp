#include "Environment.h"

Environment::Environment()
{
    window = std::make_unique<sf::RenderWindow>(sf::VideoMode(1000,1000), "Turtlebot!");
    world = std::make_unique<b2World>(b2Vec2(0.0f, 0.0f)); 
}


void Environment::draw()
{
    while(window->isOpen())
    {
        sf::Event event;
        while (window->pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
            {
                window->close();
            }
        }   
        window->clear();
        // window.draw(shape);
        window->display();
    }
}