
#import <Cocoa/Cocoa.h>
#import <Ogre.h>
#import <OgreOSXCocoaView.h>
#define __glew_h__
#include <OgreOSXCocoaWindow.h>
using namespace Ogre;


#if OGRE_VERSION_MINOR >= 12 
#define COCOA_WINDOW CocoaWindow
#else
#define COCOA_WINDOW OSXCocoaWindow
#endif

static int _frameWidth=0, _frameHeight=0;
Ogre::Rect getWindowBounds(Ogre::RenderWindow *renderWindow)
{
    Ogre::COCOA_WINDOW *cw = static_cast<Ogre::COCOA_WINDOW*>(renderWindow);
    NSWindow *win = cw->ogreWindow();
 
    // invert the y value to get the distance from the top of the display. ugh. 
    NSScreen *screen = NSScreen.mainScreen;
    NSRect rect = screen.frame;
	_frameWidth=win.frame.size.width;
	_frameHeight=win.frame.size.height;
 
    return Ogre::Rect(win.frame.origin.x, rect.size.height - win.frame.origin.y, win.frame.size.width, win.frame.size.height);
}

int getActiveWindowOSX(){
    NSAutoreleasePool *pool=[[NSAutoreleasePool alloc] init];
    NSApplication *myApp;
    myApp=[NSApplication sharedApplication];
    NSWindow *myWindow = [myApp keyWindow];
    [pool release];
//    return *NSWindow;
}
std::string getBundlePath(){
	NSString *bundlePath = [[NSBundle mainBundle] resourcePath];
	NSString *secondParentPath = [[bundlePath stringByDeletingLastPathComponent] stringByDeletingLastPathComponent];
	NSString *thirdParentPath = [secondParentPath stringByDeletingLastPathComponent] ;
	return std::string([thirdParentPath UTF8String]);
}
// print to stdout
static void NSPrint(NSString *format, ...) {
    va_list args;
    va_start(args, format);

    NSString *string = [[NSString alloc] initWithFormat:format arguments:args];

    va_end(args);

    fprintf(stdout, "%s\n", [string UTF8String]);

    [string release];
}
std::string chooseFileMac(const char* path, const char* mask){
	// WORKING :)
	printf("%s %s\n", path, mask);
	NSWindow *keyWindow = [NSApp keyWindow];
	NSOpenPanel *panel;
	NSArray* fileTypes = [[NSArray alloc] initWithObjects:[[NSString alloc] initWithCString:mask], @"PDF", nil];
	panel = [NSOpenPanel openPanel];
	[panel setFloatingPanel:YES];
	[panel setCanChooseDirectories:NO];
	[panel setCanChooseFiles:YES];
	[panel setAllowsMultipleSelection:NO];
	[panel setAllowedFileTypes:fileTypes];
	[panel setDirectoryURL:[NSURL fileURLWithPath:[[[NSString alloc] initWithCString:path] stringByExpandingTildeInPath]]];
	int i = [panel runModal];
	if(i == NSOKButton){
		NSURL *nsurl = [[panel URLs] objectAtIndex:0];
		std::string url2([[nsurl path] UTF8String]);
printf("%s\n", url2.c_str());
		return url2;
		NSPrint(@"%s", [panel URL]);
		//return std::string([[[panel URLs] objectAtIndex:0] UTF8String]);
		printf("%s\n",[[[panel URL] absoluteString] UTF8String]);
		std::string url([[[panel URL] absoluteString] UTF8String]);
		[keyWindow makeKeyWindow];
		return url.substr( 7) ; // removing "file://"
	}
	[keyWindow makeKeyWindow];
	return std::string("");
}

NSPoint ccGetWindowOrigin(){
    NSApplication *myApp = [NSApplication sharedApplication];
    NSWindow *myWindow = [myApp keyWindow];
    NSRect myFrame = [myWindow frame];
    NSPoint theOrigin = myFrame.origin;
    return theOrigin;
};
int queryMouseX()
{

    NSPoint mouseLoc; 
    mouseLoc = [NSEvent mouseLocation]; //get current mouse position

	
    return  mouseLoc.x-ccGetWindowOrigin().x;

}
int queryMouseY()
{

    NSPoint mouseLoc; 
    mouseLoc = [NSEvent mouseLocation]; //get current mouse position

    return  mouseLoc.y-ccGetWindowOrigin().y;
;

}
int queryFrameHeight()
{
/*
    NSApplication *myApp = [NSApplication sharedApplication];
    NSWindow *myWindow = [myApp keyWindow];
    NSRect myFrame = [myWindow frame];
    return  myFrame.size.height;
*/
	return _frameHeight; // return cached because myFrame is not always available.
}
int queryFrameWidth()
{
/*
    NSApplication *myApp = [NSApplication sharedApplication];
    NSWindow *myWindow = [myApp keyWindow];
    NSRect myFrame = [myWindow frame];
    return  myFrame.size.width;
*/
	return _frameWidth;
}
