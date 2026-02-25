#ifndef RIGHT_VIEW_HPP
#define RIGHT_VIEW_HPP

#include <gui_generated/right_screen/RightViewBase.hpp>
#include <gui/right_screen/RightPresenter.hpp>

class RightView : public RightViewBase
{
public:
    RightView();
    virtual ~RightView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // RIGHT_VIEW_HPP
