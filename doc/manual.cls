%
% (c)2003-2005 Karljohan Lundin
% 
% GNU Public License version 2 or later

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{manual}[2005/02/10 Document class for H3D API Manual]
\LoadClass[a4paper,11pt]{report}
\NeedsTeXFormat{LaTeX2e}

\RequirePackage{type1cm}
\RequirePackage{ifthen}

\RequirePackage[T1]{fontenc}
\RequirePackage[latin1]{inputenc}

\RequirePackage{makeidx}
\RequirePackage[pdftex]{graphicx}
\RequirePackage{subfigure}

\RequirePackage{color}
\RequirePackage{fancyhdr}
\definecolor{colorBlack}{rgb}{.1,.1,.5}
\RequirePackage[pdftex,%
 colorlinks=true,
 urlcolor=colorBlack,       % \href{...}{...} external (URL)
 filecolor=colorBlack,     % \href{...} local file
 linkcolor=colorBlack,       % \ref{...} and \pageref{...}
 citecolor=colorBlack,
 plainpages=false,
 pagebackref,
 pdfpagemode=None,
 bookmarksopen=true]{hyperref}

\AtBeginDocument{%
  \hypersetup{%
    pdftitle={\@title},%
    pdfauthor={\@author},%
    pdfsubject={},%
    pdfkeywords={}}}

\definecolor{sensegraphicscolor}{rgb}{.2,.2,.8}

\renewcommand{\maketitle}{%
  \thispagestyle{empty}
  \setlength{\unitlength}{1mm}
  \begin{picture}(150,150)(0,0)
    \put(0,75){\textcolor{sensegraphicscolor}{\Huge\bf\scshape HAPI Manual}}
    \put(0,70){\textcolor{sensegraphicscolor}{\rule{185mm}{3mm}}}
    \put(1,65){\textcolor{sensegraphicscolor}{\large\bf\ For version 1.3.1}}
  \end{picture}\newpage}

\renewcommand{\@makechapterhead}[1]{%
  \setlength{\unitlength}{1mm}
  \begin{picture}(0,18)(0,0)
    \put(0,15){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\LARGE\bf\scshape \@chapapp{} \thechapter}}}
    \put(0,10){\textcolor{sensegraphicscolor}{\rule{185mm}{1.5mm}}}
    \put(0,0){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\Huge\bf\scshape #1}}}
  \end{picture}\par
  \vskip 40\p@
}
\renewcommand{\@makeschapterhead}[1]{%
  \setlength{\unitlength}{1mm}
  \begin{picture}(0,10)(0,0)
    \put(0,15){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\LARGE\bf\scshape \@chapapp{} \thechapter}}}
    \put(0,10){\textcolor{sensegraphicscolor}{\rule{185mm}{1.5mm}}}
    \put(0,0){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\Huge\bf\scshape #1}}}
  \end{picture}\par
  \vskip 40\p@
}

\renewcommand{\tableofcontents}{%
  \newpage
  \thispagestyle{empty}
  \setlength{\unitlength}{1mm}
  \begin{picture}(0,30)(0,0)
    \put(0,10){\textcolor{sensegraphicscolor}{\rule{185mm}{1.5mm}}}
    \put(0,0){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\Huge\bf\scshape\contentsname}}}
  \end{picture}\par
  \vskip 40\p@
  \@starttoc{toc}
}
\renewcommand{\listoftables}{%
  \newpage
  \setlength{\unitlength}{1mm}
  \begin{picture}(0,10)(0,0)
    \put(0,10){\textcolor{sensegraphicscolor}{\rule{185mm}{1.5mm}}}
    \put(0,0){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\Huge\bf\scshape\listtablename}}}
  \end{picture}\par
  \vskip 40\p@
  \@starttoc{lot}
}
\renewcommand{\listoffigures}{%
  \newpage
  \setlength{\unitlength}{1mm}
  \begin{picture}(0,10)(0,0)
    \put(0,10){\textcolor{sensegraphicscolor}{\rule{185mm}{1.5mm}}}
    \put(0,0){\makebox[105mm][l]{\textcolor{sensegraphicscolor}{\Huge\bf\scshape\listfigurename}}}
  \end{picture}\par
  \vskip 40\p@
  \@starttoc{lof}
}

\ProcessOptions 

% Vertical Settings
\voffset -1in
\topmargin 15mm
\headheight 20mm
\headsep 10mm 
\textheight 202mm
\footskip = 25mm

% Horizontal Settings
\hoffset -1in
\oddsidemargin 25mm
\evensidemargin 25mm
\marginparwidth 0mm
\textwidth 160mm

\headwidth 170mm
\lhead{}
\chead{}
\rhead{\leftmark}
\lfoot{\copyright{}\@author \today}
\cfoot{\thepage}
\rfoot{}
\pagestyle{fancy}

