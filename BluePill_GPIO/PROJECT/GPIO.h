#ifndef _GPIO_H
#define _GPIO_H
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#endif   /* _GPIO_H */