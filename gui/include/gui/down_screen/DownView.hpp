#ifndef DOWN_VIEW_HPP
#define DOWN_VIEW_HPP

#include <gui_generated/down_screen/DownViewBase.hpp>
#include <gui/down_screen/DownPresenter.hpp>

class DownView : public DownViewBase
{
public:
    DownView();
    virtual ~DownView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // DOWN_VIEW_HPP
