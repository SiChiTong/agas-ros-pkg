#ifndef PUMA_MESSAGES_H
#define PUMA_MESSAGES_H

namespace puma2 {

  enum {
  msgImportant=0,
  msgNotice=2
};

}

#define PumaMessage(level, message) \
        TRACE( message, level );

#endif /* PUMA_MESSAGES_H */
