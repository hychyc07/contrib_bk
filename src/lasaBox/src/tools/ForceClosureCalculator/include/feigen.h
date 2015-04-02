
#include "basis.h"
#include "vmblock.h"

/*--------------------------------------------------------------------*
 * Auxiliary functions for  eigen                                     *
 *--------------------------------------------------------------------*/


static int balance       /* balance a matrix .........................*/
                   (int       n,      /* size of matrix ..............*/
                    REAL *    mat[],  /* matrix ......................*/
                    REAL      scal[], /* Scaling data ................*/
                    int *     low,    /* first relevant row index ....*/
                    int *     high,   /* last relevant row index .....*/
                    int       basis   /* base of computer numbers ....*/
                   );

static int balback       /* reverse balancing ........................*/
                   (int     n,        /* Dimension of matrix .........*/
                    int     low,      /* first nonzero row ...........*/
                    int     high,     /* last nonzero row ............*/
                    REAL    scal[],   /* Scaling data ................*/
                    REAL *  eivec[]   /* Eigenvectors ................*/
                   );

static int elmhes       /* reduce matrix to upper Hessenberg form ....*/
                  (int       n,       /* Dimension of matrix .........*/
                   int       low,     /* first nonzero row ...........*/
                   int       high,    /* last nonzero row ............*/
                   REAL *    mat[],   /* input/output matrix .........*/
                   int       perm[]   /* Permutation vector ..........*/
                  );



static int elmtrans       /* copy to Hessenberg form .................*/
                    (int     n,       /* Dimension of matrix .........*/
                     int     low,     /* first nonzero row ...........*/
                     int     high,    /* last nonzero row ............*/
                     REAL *  mat[],   /* input matrix ................*/
                     int     perm[],  /* row permutations ............*/
                     REAL *  h[]      /* Hessenberg matrix ...........*/
                    );


/* ------------------------------------------------------------------ */

static int orthes     /* reduce orthogonally to upper Hessenberg form */
                 (
                  int  n,                  /* Dimension of matrix     */
                  int  low,                /* [low,low]..[high,high]: */
                  int  high,               /* submatrix to be reduced */
                  REAL *mat[],             /* input/output matrix     */
                  REAL d[]                 /* reduction information   */
                 ) ;                        /* error code              */

static int orttrans       /* compute orthogonal transformation matrix */
                   (
                    int  n,      /* Dimension of matrix               */
                    int  low,    /* [low,low]..[high,high]: submatrix */
                    int  high,   /* affected by the reduction         */
                    REAL *mat[], /* Hessenberg matrix, reduction inf. */
                    REAL d[],    /* remaining reduction information   */
                    REAL *v[]    /* transformation matrix             */
                   ) ;


static int hqrvec       /* compute eigenvectors ......................*/
                  (int     n,           /* Dimension of matrix .......*/
                   int     low,         /* first nonzero row .........*/
                   int     high,        /* last nonzero row ..........*/
                   REAL *  h[],         /* upper Hessenberg matrix ...*/
                   REAL    wr[],        /* Real parts of evalues .....*/
                   REAL    wi[],        /* Imaginary parts of evalues */
                   REAL *  eivec[]      /* Eigenvectors ..............*/
                  );


static int hqr2         /* compute eigenvalues .......................*/
                (int     vec,         /* switch for computing evectors*/
                 int     n,           /* Dimension of matrix .........*/
                 int     low,         /* first nonzero row ...........*/
                 int     high,        /* last nonzero row ............*/
                 REAL *  h[],         /* Hessenberg matrix ...........*/
                 REAL    wr[],        /* Real parts of eigenvalues ...*/
                 REAL    wi[],        /* Imaginary parts of evalues ..*/
                 REAL *  eivec[],     /* Matrix of eigenvectors ......*/
                 int     cnt[]        /* Iteration counter ...........*/
                );



static int norm_1       /* normalize eigenvectors to have one norm 1 .*/
                  (int     n,       /* Dimension of matrix ...........*/
                   REAL *  v[],     /* Matrix with eigenvektors ......*/
                   REAL    wi[]     /* Imaginary parts of evalues ....*/
                  );


int eigen               /* Compute all evalues/evectors of a matrix ..*/
          (
           int     vec,           /* switch for computing evectors ...*/
           int     ortho,         /* orthogonal Hessenberg reduction? */
           int     ev_norm,       /* normalize Eigenvectors? .........*/
           int     n,             /* size of matrix ..................*/
           REAL ** mat,           /* input matrix ....................*/
           REAL ** eivec,         /* Eigenvectors ....................*/
           REAL  * valre,         /* real parts of eigenvalues .......*/
           REAL  * valim,         /* imaginary parts of eigenvalues ..*/
           int   * cnt            /* Iteration counter ...............*/
          );




