#pragma once
class Motion;


namespace MotionUtil
{
	//여기 있는 stitch함수들은 stitch나 concat클래스들과 달리, 동작 세그먼트가 한프레임씩 겹치는 가정으로 구현되었음.

	// front.pose(frontLast)=src.pose(srcFirst)
	// front.pose(frontLast+1)=src.pose(srcFirst+1)
	// front.pose(frontLast+2)=src.pose(srcFirst+2)
	// ...
	// front.pose(frontLast+srcLast-srcFirst)=src.pose(srcLast)
	// 이런식이되도록 스티치 된다.

	// 동작의 [firstSafe, lastSafe]는 부드럽게 변형되어 붙는다.
	// [..] interval을 사용할때는 first, last
	// [..) interval을 사용할때는 start, end 로 표기한다.
	void stitchGlobal(int firstSafe, int lastSafe, Motion& front, Motion const& src, int srcFirst, int srcLast, int frontLast=INT_MAX, bool two=true);
	void stitchGlobalC2(int firstSafe, int lastSafe, Motion& front, Motion const& src, int srcFirst, int srcLast, int frontLast=INT_MAX, bool two=true);


	// 기본적으로 위와 동일하지만, 동작 뒤에 tail만큼 더 붙여놓는다. (다음에 블렌딩할 수 있도록.)
	void stitchBlend(int firstSafe, int lastSafe, Motion& front, Motion const& src, int srcFirst, int srcLast, int frontLast, int tail);

	void nostitch(Motion& front, Motion const& src, int srcfirst, int srclast);

	// 가정.
	// front의 마지막 프레임이 src.pose(transitionFrom)에 해당한다.  (완전히 동일할 필요는 없지만 최대한 유사하여야한다.)
	//
	//                                transitionFrom
	//  front...............................|
	//  src                                 |..................|
	//                                 srcFirst              srcLast
	//  searchRange               |                  |
	//					      firstSafe				lastSafe
	void stitchTransition(int firstSafe, int lastSafe, Motion& front, int transitionFrom, Motion const& src, int srcFirst, int srcLast, int frontLast=INT_MAX);



	void inpaint(int startInpaint, int endInpaint, Motion& front);

}
//////////////// deprecated. 아래에 있는 클래스들은 사용하지 말것.
#include <typeinfo>
namespace MotionUtil
{

	//여기 있는 stitch함수들은 stitch나 concat클래스들과 달리, 동작 세그먼트가 한프레임씩 겹치는 가정으로 구현되었음.
	// 이 방식이 더 최신 방식임.

	// 앞쪽으로 prevSafe만큼은 바뀌어도 된다. 뒤쪽으로 afterSafe만큼은 바뀌어도 된다.
	// front 에 add를 부드럽게 연결한다.
	void stitchLocal(int prevSafe, int afterSafe, Motion& front, Motion const& add);
	void stitchGlobal(int prevSafe, int afterSafe, Motion& front, Motion const& add);
	void stitchOnline(Motion& front, Motion const& add);


	/// Stitch는 discontinuity를 없애는 방식으로 이어붙인다.
	class Stitch
	{
	public:
		Stitch(){}
		virtual ~Stitch(){}


//
//		에러가 양방향으로 propagate된다.
//		 * \param safe
//		 주변으로 error를 propagate해도 되는 범위이다.
//		 front.length()+-safe가 꼭 valid index라는 보장이 없으므로, 상속한 클래스에서 구현시, valid index가 아니어도 에러가 나지 않도록
//		 구현할것.
//
		virtual void stitch(int safe, Motion& front, const Motion& add)
			{ Msg::error("%s is not implemented", typeid( *this).name()); }


		//
		//에러가 뛰쪽으로만 propagate된다.
		 //* \param safe
		 //뒤쪽으로 error를 propagate해도 되는 범위이다.
		 //front.length()+-safe가 꼭 valid index라는 보장이 없으므로, 상속한 클래스에서 구현시, valid index가 아니어도 에러가 나지 않도록
		 //구현할것.
		 //
		virtual void stitchOnline(int safe, Motion& front, const Motion& add)
			{ Msg::error("%s is not implemented", typeid( *this).name()); }
	};


	class Concat;
	// C1 continuous stitch- alwas bettern than C0stitch above.
	class C1stitch : public Stitch
	{
		const Concat& m_concator;
	public:
		C1stitch(const Concat& concator):m_concator(concator){}
		virtual ~C1stitch(){}

		virtual void stitch(int safe, Motion& front, const Motion& add);
		virtual void stitchOnline(int safe, Motion& front, const Motion& add);
	};

	// C1 continuous root-special (root is specially stitched)
	class C1stitch2 : public Stitch
	{
		const Concat& m_concator;
	public:
		C1stitch2(const Concat& concator):m_concator(concator){}
		virtual ~C1stitch2(){}

		virtual void stitch(int safe, Motion& front, const Motion& add);
		static void stitchUtilRoot(int safe, Motion& afterConcat, int numFrameOld);
		static void stitchUtilJoint(int safe, Motion& afterConcat, int numFrameOld);
		static void stitchUtil(int safe, Motion& afterConcat, int numFrameOld);
	};

	namespace linstitch
	{
		void stitchUtilRoot(int safe, Motion& afterConcat, int numFrameOld);
		void stitchUtilJoint(int safe, Motion& afterConcat, int numFrameOld);
	}
	// C1 continuous stitch using stitching the displacementMap.
	class C1stitchReconstruct : public Stitch
	{
	public:
		C1stitchReconstruct (){}
		virtual ~C1stitchReconstruct (){}

        virtual void stitch(int safe, Motion& front, const Motion& add);
	};

	class StitchRetarget: public Stitch
	{
	public:
		StitchRetarget(){}
		virtual ~StitchRetarget(){}
        virtual void stitch(int safe, Motion& front, const Motion& add);
	};

	class NoStitch: public Stitch
	{
		const Concat& m_concator;
	public:
		NoStitch(const Concat& concator):m_concator(concator){}
		virtual ~NoStitch(){}

		virtual void stitch(int safe, Motion& front, const Motion& add);
	};

	namespace C0stitch
	{
		void stitchUtil(int safe, Motion& front, int numFrameOld);
	};

}
