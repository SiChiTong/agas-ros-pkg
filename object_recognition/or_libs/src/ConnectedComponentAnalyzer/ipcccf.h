/* 

This "SOFTWARE" is a free software.

You are allowed to download, use, modify and redistribute this software.
The software is provided "AS IS" without warranty of any kind.

Copyright: University of Koblenz, Patrick Sturm

*/


/*
  CPluginImage* input = dynamic_cast<CPluginImage*>(pd);
  if (input && input->GetFormat()=="Matrix<GRAY>")
{
          CPluginImage* output=new CPluginImage(input->GetWidth(),input->GetHeight(),
    (unsigned char*)new unsigned[input->GetWidth()*input->GetHeight()],
    4,true,"Matrix<GRAY>");
          if (input->GetBytesPerPixel()==1)
{
    ipcConnectedComponentFilter<unsigned char,unsigned> ccf8bit;
    ccf8bit.init(input->GetWidth(),input->GetHeight());
    ccf8bit.execute(input->GetData(),(unsigned*)output->GetData());
}
*/


#ifndef _ipcccf_h_
#define _ipcccf_h_

/***********************************************
iMAge pROcessing cLAsses Version 1.0

Copyright 2003, Patrick Sturm
************************************************/

// Include-Files
template <typename ipcLabelType>
class ipcEqClasses
{
protected:
	unsigned size,delta,maxLabel;
	ipcLabelType* eqClasses;

	void enlarge(unsigned newSize);
	void enlarge();
public:
	ipcEqClasses();
	virtual ~ipcEqClasses();

	inline void init(int maxNumberOfLabels, float initPercentage = 0.25, float deltaPercentage = 0.1);
	inline ipcLabelType resolve();
	inline void insert(ipcLabelType label1, ipcLabelType label2);
	inline void clear();
	inline ipcLabelType newLabel();

	const ipcLabelType* getEQClasses() const { return eqClasses; };
};

template <typename ipcPixelType, typename ipcLabelType>
class ipcConnectedComponentFilter: protected ipcEqClasses<ipcLabelType>
{
private:
	int w,h,d,numOfPixels;

public:
	ipcConnectedComponentFilter();
	virtual ~ipcConnectedComponentFilter() {};
	inline void init(int width, int height, int depth=1);
	ipcLabelType execute(const ipcPixelType* input, ipcLabelType* labelImage);
};

template <typename ipcLabelType>
struct AlwaysValid
{
  bool operator()(int, ipcLabelType) const
  { return true; }
};

template <typename ipcPixelType, typename ipcLabelType, typename ipcSimilarity, typename ipcValidity=AlwaysValid<ipcLabelType> >
class ipcConnectedComponentFilterSim: protected ipcEqClasses<ipcLabelType>
{
private:
	int w,h,numOfPixels;

public:
	ipcConnectedComponentFilterSim();
	virtual ~ipcConnectedComponentFilterSim() {};
	inline void init(int width, int height);
	ipcLabelType execute(const ipcPixelType* input, ipcLabelType* labelImage, const ipcSimilarity& sim,
                       const ipcValidity& valid=ipcValidity(), int maxr=1);
};

//------------------------------------------------------------------------

template <typename ipcLabelType>
ipcEqClasses<ipcLabelType>::ipcEqClasses()
{
	maxLabel=0;
	eqClasses=NULL;
	size=0;
	maxLabel=0;
}

template <typename ipcLabelType>
ipcEqClasses<ipcLabelType>::~ipcEqClasses()
{
	if (eqClasses) delete[] eqClasses;
}

template <typename ipcLabelType>
inline void ipcEqClasses<ipcLabelType>::init(int maxNumberOfLabels, float initPercentage, float deltaPercentage)
{
	maxLabel=0;
	delta=(unsigned)(deltaPercentage*maxNumberOfLabels);
	enlarge((unsigned)(initPercentage*maxNumberOfLabels));
	eqClasses[0]=0;
}

template <typename ipcLabelType>
inline void ipcEqClasses<ipcLabelType>::enlarge(unsigned newSize)
{
	if (newSize>size)
	{
		ipcLabelType* newEQClasses = new ipcLabelType[newSize];
		if (eqClasses)
		{
			memcpy(newEQClasses,eqClasses,sizeof(ipcLabelType)*(maxLabel+1));
			delete[] eqClasses;
		}
		eqClasses=newEQClasses;
		size=newSize;
	}
}

template <typename ipcLabelType>
inline void ipcEqClasses<ipcLabelType>::enlarge()
{
	enlarge(size+delta);
}

template <typename ipcLabelType>
inline void ipcEqClasses<ipcLabelType>::insert(ipcLabelType label1, ipcLabelType label2)
{
	ipcLabelType tmp;

	if (label1==label2) return;

	if (label1<label2)
	{
		tmp=label1;
		label1=label2;
		label2=tmp;
	}

	while (eqClasses[label1]!=label2)
	{
		if (eqClasses[label1]==label1)
			eqClasses[label1]=label2;
		else if (eqClasses[label1]<label2)
		{
			tmp=eqClasses[label1];
			label1=label2;
			label2=tmp;
		}
		else if (eqClasses[label1]>label2)
		{
			tmp=eqClasses[label1];
			eqClasses[label1]=label2;
			label1=tmp;
		}
	}

	/*if (eqClasses[label1]==label1)
		eqClasses[label1]=label2;
	else if (eqClasses[label1]<label2)
		insert(label2,eqClasses[label1]);
	else
	{
		tmp=eqClasses[label1];
		eqClasses[label1]=label2;
		insert(tmp,label2);
	}*/
}

template <typename ipcLabelType>
inline ipcLabelType ipcEqClasses<ipcLabelType>::resolve()
{
	ipcLabelType label = 0;
	for (ipcLabelType i=0; i<=maxLabel; ++i)
	{
		if (i==eqClasses[i]) eqClasses[i]=label++;
		else eqClasses[i]=eqClasses[eqClasses[i]];
	}
	return label;
}

template <typename ipcLabelType>
inline void ipcEqClasses<ipcLabelType>::clear()
{
	maxLabel=0;
}

template <typename ipcLabelType>
inline ipcLabelType ipcEqClasses<ipcLabelType>::newLabel()
{
	maxLabel++;
	if (maxLabel>=size) enlarge();
	eqClasses[maxLabel]=maxLabel;

	return maxLabel;
}

//-----------------------------------------------------------

template <typename ipcPixelType, typename ipcLabelType>
ipcConnectedComponentFilter<ipcPixelType, ipcLabelType>::ipcConnectedComponentFilter()
{
	w=h=0;
	d=1;
}


template <typename ipcPixelType, typename ipcLabelType>
void ipcConnectedComponentFilter<ipcPixelType, ipcLabelType>::init(int width, int height, int depth)
{
	w=width;
	h=height;
	d=depth;
	numOfPixels=w*h;
	ipcEqClasses<ipcLabelType>::init(numOfPixels,0.25,0.25);
}


template <typename ipcPixelType, typename ipcLabelType>
ipcLabelType ipcConnectedComponentFilter<ipcPixelType, ipcLabelType>::execute(const ipcPixelType* input, ipcLabelType* labelImage)
{
	int x,y,z,n,off,noff;
	int delta[4] = {-1-w, -w, 1-w , -1};
	int deltaX[4] = {-1,0,1,-1};
	int deltaY[4] = {-1,-1,-1,0};
	ipcLabelType labels[4];
	int numOfLabels;
	ipcLabelType minLabel;
	ipcLabelType maxLabel=0;

	off=0;
	for (z=0; z<d; ++z)
	{
		for (y=0; y<h; ++y)
		{
			for (x=0; x<w; ++x)
			{
				if (input[off]>0)
				{
					numOfLabels=0;
					for (n=0; n<4; ++n)
					{
						if (x+deltaX[n]>=0 && x+deltaX[n]<w && y+deltaY[n]>=0 && y+deltaY[n]<h)
						{
							noff=off+delta[n];
							if (input[off]==input[noff])
								labels[numOfLabels++]=labelImage[noff];
						}
					}
					if (numOfLabels>0)
					{
						minLabel=labels[0];
						for (n=1; n<numOfLabels; ++n)
							if (labels[n]<minLabel) minLabel=labels[n];
					}
					else minLabel=this->newLabel();
					labelImage[off]=minLabel;
					if (labelImage[off]>maxLabel) maxLabel=labelImage[off];
					for (n=0; n<numOfLabels; ++n)
						insert(minLabel,labels[n]);
				}
				else labelImage[off]=0;
				off++;
			}
		}
	}
	this->resolve();
	for (off=0; off<numOfPixels; ++off)
		labelImage[off]=this->eqClasses[labelImage[off]];
	return (ipcLabelType)maxLabel;
}

//-----------------------------------------------------------

template <typename ipcPixelType, typename ipcLabelType, typename ipcSimilarity, typename ipcValidity>
ipcConnectedComponentFilterSim<ipcPixelType, ipcLabelType, ipcSimilarity, ipcValidity>::ipcConnectedComponentFilterSim()
{
	w=h=0;
}


template <typename ipcPixelType, typename ipcLabelType, typename ipcSimilarity, typename ipcValidity>
void ipcConnectedComponentFilterSim<ipcPixelType, ipcLabelType, ipcSimilarity, ipcValidity>::init(int width, int height)
{
	w=width;
	h=height;
	numOfPixels=w*h;
	ipcEqClasses<ipcLabelType>::init(w*h,0.25,0.25);
}

template <typename ipcPixelType, typename ipcLabelType, typename ipcSimilarity, typename ipcValidity>
ipcLabelType ipcConnectedComponentFilterSim<ipcPixelType, ipcLabelType, ipcSimilarity, ipcValidity>::execute(const ipcPixelType* input, ipcLabelType* labelImage, const ipcSimilarity& sim, const ipcValidity& valid, int maxr)
{
	int x,y,n,off,noff;
	int delta[4] = {-1-w, -w, 1-w , -1};
	int deltaX[4] = {-1,0,1,-1};
	int deltaY[4] = {-1,-1,-1,0};
	ipcLabelType labels[4];
	int numOfLabels;
	ipcLabelType minLabel;

	off=0;
	for (y=0; y<h; ++y)
	{
		for (x=0; x<w; ++x)
		{
			if (valid(off,input[off]))
			{
				numOfLabels=0;
				for (n=0; n<4; ++n)
				{
					if (x+deltaX[n]>=0 && x+deltaX[n]<w && y+deltaY[n]>=0 && y+deltaY[n]<h)
					{
						noff=off+delta[n];
						if (valid(noff,input[noff]) && sim(input[off],input[noff]))
							labels[numOfLabels++]=labelImage[noff];
					}
				}
				if (numOfLabels>0)
				{
					minLabel=labels[0];
					for (n=1; n<numOfLabels; ++n)
						if (labels[n]<minLabel) minLabel=labels[n];
				}
				else
        {
          // Robocup 2010 HACK. Old:
          //
          // minLabel=this->newLabel();
          //
          // New: check all neighbors within certain distance for similarity instead of
          // direct neighbors. If this works, make it part of the real code.. and faster
          for ( int rmax = 2; rmax < maxr; ++rmax )
          {
            for ( int r = 2; r <= rmax; ++r )
            {
              for ( int ny = y-r; ny <= y+r; ++ny )
              {
                for ( int nx = x-r; nx <= x+r; ++nx )
                {
                  if ( ny < 0 || ny >= h || nx < 0 || nx >= w )
                    continue;
                  noff = ny*w+nx;
                  if ( !valid( noff, input[noff] ) )
                    continue;
                  if ( sim( input[off], input[noff] ) )
                  {
                    if ( numOfLabels < 4 )
                      labels[numOfLabels++]=labelImage[ny*w+nx];
                    else if ( labelImage[ny*w+nx] < labels[3] )
                      labels[3]=labelImage[ny*w+nx];
                  } // if
                } // for nx
              } // for ny
              
              if ( numOfLabels > 0 )
              {
                break;
              }
            } // for r
            if ( numOfLabels > 0 )
            {
              break;
            }
          } // for rmax
          if ( numOfLabels > 0 )
          {
            minLabel=labels[0];
            for (n=1; n<numOfLabels; ++n)
              if (labels[n]<minLabel) minLabel=labels[n];
          }
          else minLabel = this->newLabel();
        } // else
				labelImage[off]=minLabel;
				if (labelImage[off]==1)
					off=off;
				for (n=0; n<numOfLabels; ++n)
					insert(minLabel,labels[n]);
			}
			else labelImage[off]=0; // not a valid pixel
			off++;
		}
	}
	ipcLabelType maxLabel = 0;
	this->resolve();
	for (off=0; off<numOfPixels; ++off)
	{
		labelImage[off]=this->eqClasses[labelImage[off]];
		if (labelImage[off]>maxLabel) maxLabel=labelImage[off];
	}
	return maxLabel;
}

#endif
