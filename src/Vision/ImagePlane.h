#pragma once

class ImagePlane {
	unsigned int texObj;
	float *mImagePlaneVertices;
	float *mImagePlaneTexcoords;
	unsigned char *mImagePlaneIndices;
	int mFrameWidth;
	int mFrameHeight;
	int mTexWidth;
	int mTexHeight;
	float mInitialRectLeft;
	float mInitialRectRight;
	float mInitialRectBottom;
	float mInitialRectTop;
	float *mInitialRect;
	float *mInitialRegionRest;
	unsigned char *mInitialRegionRestIndices;
	bool bShowInitialRegion;

public:
	void ShowInitialRegion(bool show) {
		bShowInitialRegion = show;
	}

	bool IsVisibleInitialRegion() {
		return bShowInitialRegion;
	}

	ImagePlane(int width, int height);

	~ImagePlane(void);

	void SetImage(void *imageData);

	void Draw();
	void Draw2();
};
