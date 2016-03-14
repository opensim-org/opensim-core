#include <iostream>
using namespace std;

class AbstractConnector {
public:
    virtual std::string getTypeName() const = 0;
};

template <typename T>
class Connector : public AbstractConnector {
public:
    std::string getTypeName() const override {
        return T::getClassName();
    }
};

class Component {
public:
    virtual void VIRTUAL() = 0;
protected:
    template <typename T> int constructConnector() {
        aconn = new Connector<T>();
        return 0;
    }
private:
    AbstractConnector* aconn;
};

class Bar;

class Foo : public Component {
public:
    virtual void VIRTUAL() {}
    //int z { constructConnector<Bar>() };
    void constructConnector_bar() {
        constructConnector<Bar>();
    }
    static std::string getClassName() { return "Foo"; }
    
};

/*
class Bar : public Component {
public:
    virtual void VIRTUAL() {}
    int x { constructConnector<Foo>() };
    static std::string getClassName() { return "Bar"; }
    
};
*/


int main() {
    cout << "hello " << endl;
}