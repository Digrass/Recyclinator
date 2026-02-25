#ifndef WAITVIEW_HPP
#define WAITVIEW_HPP

#include <gui_generated/wait_screen/WaitViewBase.hpp>
#include <gui/wait_screen/WaitPresenter.hpp>

class WaitView : public WaitViewBase
{
public:
    WaitView();
    virtual ~WaitView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void screenClicked() override;
protected:
};

#endif // WAITVIEW_HPP
