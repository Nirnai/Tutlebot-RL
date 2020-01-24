#include <memory>
#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>

class Environment
{
    public:
        Environment();
        void draw();  


    private:
        std::unique_ptr<b2World> world; 
        std::unique_ptr<sf::RenderWindow> window;  
        float frame_rate;
        float step_size;
};  