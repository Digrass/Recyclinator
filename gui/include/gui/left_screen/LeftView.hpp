#ifndef LEFT_VIEW_HPP
#define LEFT_VIEW_HPP

#include <gui_generated/left_screen/LeftViewBase.hpp>
#include <gui/left_screen/LeftPresenter.hpp>

class LeftView : public LeftViewBase
{
public:
    LeftView();
    virtual ~LeftView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // LEFT_VIEW_HPP
