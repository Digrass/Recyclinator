#ifndef UP_VIEW_HPP
#define UP_VIEW_HPP

#include <gui_generated/up_screen/UpViewBase.hpp>
#include <gui/up_screen/UpPresenter.hpp>

class UpView : public UpViewBase
{
public:
    UpView();
    virtual ~UpView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // UP_VIEW_HPP
