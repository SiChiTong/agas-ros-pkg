#ifndef ImageSuperClasses_H
#define ImageSuperClasses_H

namespace puma2 {

class Image {
  protected:
    /** This function initializes the defaults for the variables mValueRangeMinimum
     * and mValueRangeMaximum by setting them to the default value as delivered
     * by calling getValueRangeMinimum() and getValueRangeMaximum() for the
     * ElementType of the current image class.
     * It must be called by every constructor of classes derived from Image, as
     * I didn't find a way to inherit this behaviour "down the chain" :-(
     */
    void setupImageBaseVariables();
  protected:
    /**
     * Holds the intended minimum value any sample of this image should have.
     * This is not the current minimal value of all samples, but the value
     * according to the range expected, like an 8-bit gray level image usually
     * has an expected range from 0 (mValueRangeMinimum) to 255 (mValueRangeMaximum),
     * whereas a 4-bit image has a range of 0..15, 12-bit has 0..4095 and so on.
     */
    double mValueRangeMinimum;

    /**
     * Holds the intended maximum value any sample of this image should have.
     * This is not the current minimal value of all samples, but the value
     * according to the range expected, like an 8-bit gray level image usually
     * has an expected range from 0 (mValueRangeMinimum) to 255 (mValueRangeMaximum),
     * whereas a 4-bit image has a range of 0..15, 12-bit has 0..4095 and so on.
     */
    double mValueRangeMaximum;

  public:
    /**
     * Constructor
     */
    Image();

    /**
     * Destructor
     */
    virtual ~Image();

    /**
     * Tell about the minimal value which can be stored by an element (a sample)
     * which is the base type of this image class.lementTypeM
     * (This should be some kind of constant, individually set for each
     *  subclass, but C++ allows such overloading only for functions,
     *  so we have to implement it as a constant function.)
     */
    virtual double getElementTypeMinimum() const = 0;

    /**
     * Tell about the maximal value which can be stored by an element (a sample)
     * which is the base type of this image class.
     * (see getElementTypeMaximum().)
     */
    virtual double getElementTypeMaximum() const = 0;

    /** set mValueRangeMinimum to value
     * @param[in] value new minimum */
    void setValueRangeMinimum(double value) { mValueRangeMinimum = value; };
    /** set mValueRangeMaximum to value
     * @param[in] value new maximum */
    void setValueRangeMaximum(double value) { mValueRangeMaximum = value; };
    /** get mValueRangeMinimum */
    double getValueRangeMinimum() const { return mValueRangeMinimum; };
    /** get mValueRangeMaximum */
    double getValueRangeMaximum() const { return mValueRangeMaximum; };

    /** Read in an image from a given file.
     * This usually is a convenience shortcut, in fact the class ImageReader
     * will be employed to do the real work.
     * @param[in] fileName  name of image file */
    void readFromFile(const char * fileName);

    /** Write this image to a file.
     * This usually is a convenience shortcut, in fact the class ImageWriter
     * will be employed to do the real work.
     * The file type to be written is determined from the file name suffix.
     * @param[in] fileName  name of image file */
    void writeToFile(const char * fileName) const;
};

}


#endif
