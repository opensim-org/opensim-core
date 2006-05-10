Version: $Id: README.txt,v 1.2 2002/06/10 20:09:55 cxh Exp $
AElfred, Version 1.1
Microstar's Java-Based XML Parser
Copyright (c) 1997, 1998 by Microstar Software Ltd.
Home Page: http://www.microstar.com/XML/

AElfred is free for both commercial and non-commercial use and
redistribution, provided that Microstar's copyright and disclaimer are
retained intact.  You are free to modify AElfred for your own use and
to redistribute AElfred with your modifications, provided that the
modifications are clearly documented.


DISCLAIMER
----------

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
merchantability or fitness for a particular purpose.  Please use it AT
YOUR OWN RISK.


** NEWS **
----------

Version 1.2a:

- AElfred has a new maintainer.  Side-effects of this will include
  a gradual change in the indenting style of the source code, a
  mixture of Unix and MSDOS-style text files, and other things that
  people who actually use the source code will find annoying.
  
  The transition will be complete at the next numbered release.

- Source code directory tree has changed to reflect JDK 1.2 requirements.

- Other small tweaks to allow compilation under JDK 1.2.

- No longer crashes when reading longish files from a Reader.

- No longer hangs when syntax errors appear after the document element.

- aelfred.jar is no longer out of date :-).

- XmlApp, the base class for the included demos, no longer extends Applet.
  The new class, XmlApplet, now works both as an applet and application
  like XmlApp used to.  JDK 1.2 beta 3 had problems with this behaviour --
  if an Applet was constructed (but not used) in an app, its finalization
  would hang the JVM.  This change will persist even after the JDK bug is
  fixed.
  
  The demos, therefore, no longer work as applets.

- Performance should be somewhat improved.

Version 1.2:

- AElfred can parse from an InputStream, Reader, or a system ID
- resolveEntity() can return an InputStream or Reader as well as a
  system ID
- new SAX driver for SAX 1.0gamma (http://www.megginson.com/SAX/)


DESIGN PRINCIPLES
-----------------

In most Java applets and applications, XML should not be the central
feature; instead, XML is the means to another end, such as loading
configuration information, reading meta-data, or parsing transactions.

When an XML parser is only a single component of a much larger
program, it cannot be large, slow, or resource-intensive.  With Java
applets, in particular, few programmers would be willing to require
users to download an additional 50K to 100K in class files just for
the sake of adding XML capability.

AElfred is a new Java-based XML parser from Microstar Software
Ltd. (http://www.microstar.com/), an established provider of XML and
SGML solutions.  AElfred is distributed for free (with full source) for
both commercial and non-commercial use, and is designed for easy and
efficient use over the Internet, based on the following principles:

1. AElfred must be as small as possible, so that it doesn't add too
   much to your applet's download time.

   STATUS: AElfred is currently about 26K in total.  The compressed
    JAR file, including the optional classes, is only 15K.

2. AElfred must use as few class files as possible, to minimize the number
   of HTTP connections necessary.

   STATUS: AElfred consists of only two core class files, the main
    parser class (XmlParser.class) and a small interface for your own
    program to implement (XmlHandler.class).  All other classes in
    the distribution are either optional or for demonstration only.

3. AElfred must be compatible with most or all Java implementations
   and platforms.

   STATUS: AElfred uses mainly JDK 1.0.2 features (you may parse from
    a JDK 1.1 Reader if you wish), and we have tested it successfully
    with the following Java implementations: JDK 1.1.1 (Linux), jview
    (Windows NT), Netscape 4 (Linux and Windows NT), Internet Explorer
    3 (Windows NT), and Internet Explorer 4 (Windows NT).

4. AElfred must use as little memory as possible, so that it does not take
   away resources from the rest of your program.

   STATUS: On a P75 Linux system, using JDK 1.1.1, running AElfred
    (with a 4MB XML document) takes only 2MB more memory than running
    a simple "Hello world" Java application.  Because AElfred does not
    build an in-memory parse tree, you can run it on very large input
    files using little or no extra memory.

5. AElfred must run as fast as possible, so that it does not slow down
   the rest of your program.

   STATUS: On a P75 Linux system, using JDK 1.1.1 (without a JIT
    compiler), AElfred parses test files at about 70K/second.  On a
    P166 NT workstation, using jview, AElfred parses test files at
    about about 1MB/second.

6. AElfred must produce correct output for well-formed and valid
   documents, but need not reject every document that is not valid or
   not well-formed.

   STATUS: AElfred is DTD-aware, and handles all current XML features,
    including CDATA and INCLUDE/IGNORE marked sections, internal and
    external entities, proper whitespace treatment in element content,
    and default attribute values.  It will sometimes accept input that
    is technically incorrect, however, without reporting an error (see
    below), since full error reporting would make the parser larger.

7. AElfred must provide full internationalisation from the first release.

   STATUS: AElfred supports Unicode to the fullest extent possible in
    Java.  It correctly handles XML documents encoded using UTF-8,
    UTF-16, ISO-10646-UCS-2, ISO-10646-UCS-4 (as far as surrogates
    allow), and ISO-8859-1 (ISO Latin 1/Windows).  With these
    character sets, AElfred can handle all of the world's major (and
    most of its minor) languages.  To support other character
    encodings, you can supply your own Reader.

As you can see from this list, AElfred is designed for production use,
not for validation: it will behave correctly with correct files, but
it will not necessarily let you know when there are errors.  If you
want to verify on your system that your XML document is valid or
well-formed before you publish it to the Internet, then you should use
a validating XML parser.


SAX
---

AElfred now has a SAX 1.0 (gamma) driver built-in to the main
distribution, so there is no need to use an external driver.  The
class name of the driver is

  com.microstar.xml.driver.SAXDriver

[Ptolemy Note:  We moved SAXDriver.java into the driver subdirectory
because 'make fast' runs javac *.java, which was causing problems
because SAXDriver uses SAX, which we do not ship]

STARTUP
-------

To test the parser, you can try one of the demonstration applications
EventDemo, TimerDemo, DtdDemo, or StreamDemo (see descriptions below).
To start an application, make certain that AElfred's classes are
somewhere on your CLASSPATH, then use a command line like the
following:

  java EventDemo <uri>

Where <uri> can be relative (i.e. a file in the current directory) or
absolute (i.e. an XML document anywhere on the Internet).  Note that
StreamDemo takes a file name rather than a URI.

XML parsers like AElfred read one or more source files (called
"entities"), and merge them into a single stream of logical events,
such as character data, processing instructions, or the start or end
of elements.  The EventDemo demonstration application gives you an
idea of what information appears in the stream.


PROGRAMMER'S USAGE
------------------

To use AElfred in your own Java applets or applications, you need to
supply a class that implements the com.microstar.xml.XmlHandler
interface (say, "MyHandler"), create a new com.microstar.xml.XmlParser
object (supplying the URI of the XML document), then explicitly start
a parse:

  import com.microstar.xml.XmlParser;

  [...]

  String uri = "http://www.host.com/sample.xml";
  MyHandler handler = new MyHandler();
  XmlParser parser = new XmlParser();

  parser.setHandler(handler);
  parser.parse(uri, null, null);

During the parse, AElfred will call methods in your handler to report
events; for example, whenever a new element begins, it will call your
method

  void startElement(String name)

There are thirteen callback methods altogether in the XmlHandler
interface; for details, see HTML/com.microstar.xml.XmlHandler.html:

public void startDocument ();
public void endDocument ();
public String resolveEntity (String ename, String publicId, String systemId);
public String startExternalEntity (String systemId);
public String endExternalEntity (String systemId);
public void doctypeDecl (String dname, String publicId, String systemId);
public void attribute (String name, String value, boolean isSpecified);
public void startElement (String elname);
public void endElement (String elname);
public void charData (char ch[], int start, int length);
public void ignorableWhitespace (char ch[], int start, int length);
public void processingInstruction (String target, String data);
public void error (String message, String url, int line, int column);

If you want to avoid implementing all of these, you can derive your
handler from com.microstar.xml.HandlerBase (see below), then fill in
only the ones you need.


REQUIRED CLASSES
----------------

If you want to use the XML parser with a Java applet, these are the
only two classes that you must install (in addition to your own class
that implements the XmlHandler interface):

com/microstar/xml/XmlParser.class              
                        The actual XML parser.

com/microstar/xml/XmlHandler.class      
                        The callback interface for the XML parser.


OPTIONAL CLASSES
----------------

There are three optional classes that you may choose to install on a
web site (all are very small):

com/microstar/xml/XmlException.class
                        A convenience class for reporting XML
                        parsing errors.

com/microstar/xml/HandlerBase.class
                        (Requires XmlException.class.) A base class
                        that provides a default implementation for all
                        the handlers.

com/microstar/xml/driver/SAXDriver.class
                        A SAX driver for AElfred.

[Ptolemy Note:  We moved SAXDriver.java into the driver subdirectory
because 'make fast' runs javac *.java, which was causing problems
because SAXDriver uses SAX, which we do not ship]

Neither of the required classes refers to these, and you may leave
them out of a web installation if you do not need them.


DEMONSTRATION CLASSES
---------------------

The distribution also contains some classes for the demonstration
applications.  Normally, you will not include these in a web
installation:

XmlApp.class            A simple base class for the demonstrations.

EventDemo.class         (Requires XmlApp.class.) A demonstration
                        application showing the XML parse events.

TimerDemo.class         (Requires XmlApp.class.) A demonstration
                        application that simply parses a document
                        without producing any running output (except
                        for errors and warnings); useful for timing
                        the parser on different documents.

DtdDemo.class           (Requires XmlApp.class.) A demonstration
                        application that parses a document and prints
                        out a normalised version of the document's DTD
                        (if any).

StreamDemo.class        (Requires XmlApp.class.)  A demonstration
                        application that parses a document from an
                        input stream rather than a URL.


PARSER QUERY METHODS
--------------------

In addition to receiving callbacks, your XmlHandler object can send
queries about the DTD back to the XmlParser object.  The results of
these queries are not guaranteed complete until after your handler has
received a doctypeDecl() event (for details of these query methods,
see HTML/com.microstar.xml.XmlParser.html):

package com.microstar.xml.XmlParser;

public Enumeration declaredElements ()
public int getElementContentType (String elname)
public String getElementContentModel (String elname)

public Enumeration declaredAttributes (String elname)
public int getAttributeType (String elname, String aname)
public String getAttributeEnumeration (String elname, String aname)
public String getAttributeDefaultValue (String elname, String aname)
public String getAttributeExpandedValue (String elname, String aname)
public String getAttributeDefaultValueType (String elname, String aname)

public Enumeration declaredEntities ()
public int getEntityType (String ename)
public String getEntityPublicId (String ename)
public String getEntitySystemId (String ename)
public String getEntityValue (String ename)
public String getEntityNotationName (String ename)

public Enumeration declaredNotations ()
public String getNotationPublicId (String nname)
public String getNotationSystemId (String nname)


CHARACTER ENCODINGS
-------------------

** WARNING: AS REQUIRED BY THE XML SPECIFICATION, AELFRED ASSUMES THAT
            8-BIT CHARACTERS USE UTF-8 ENCODING UNLESS YOU PROVIDE AN
            ENCODING DECLARATION.  UTF-8 IS NOT THE SAME AS
            ISO-8859-1, SO 8-BIT ACCENTED CHARACTERS WILL CAUSE AN
            ERROR UNLESS YOU EXPLICITLY DECLARE "ISO-8859-1" AS YOUR
            ENCODING.

The AElfred parser currently supports all of the following input
encodings:

UTF-8                   The 8-bit Unicode transformation encoding.
UTF-16                  The 16-bit Unicode transformation encoding
                        (two octet orders).
ISO-10646-UCS-2         The standard 16-bit Unicode encoding (two
                        octet orders).
ISO-10646-UCS-4         The standard 32-bit Unicode encoding (four
                        octet orders).
ISO-8859-1              The 8-bit standard used in HTML and by Windows
                        and Unix.

If you need only 7-bit characters (i.e. ASCII), you can declare the
encoding as either "UTF-8" or "ISO-8859-1":

  <?xml version="1.0" encoding="ISO-8859-1"?>

If you use ISO-8859-1, the parser will run slightly faster, since it
doesn't need to do any character transformation on input.  If you do
not declare an encoding, then the parser will (for 7- or 8-bit text)
default to UTF-8, which will work for ASCII but is slightly slower.

Since Java has only 16-bit characters, the parser does not resolve
surrogate pairs in UTF-16; as a result, UTF-16 and UCS-2 are currently
identical.


ERRORS AND WARNINGS
-------------------

AElfred should always produce correct output for valid and well-formed
XML documents.  Since AElfred is not a fully conforming XML parser,
however, it will sometimes accept documents that are not well-formed
or are not valid, without signaling an error.

For example, James Clark has a error-reporting test suite available at

  ftp://ftp.jclark.com/pub/test/xmltest.zip

These tests cover only well-formedness, not validity.  With the 1997
version of this package, out of 141 tests AElfred _failed_ to report
errors for 31 of them:

002.xml                 Illegal character at the start of a NAME.
006.xml                 '--' in comment.
014.xml                 '<' in attribute value
023.xml                 Illegal character at the start of a NAME.
024.xml                 Illegal character at the start of a NAME.
025.xml                 ']]>' in PCDATA.
026.xml                 ']]>' in PCDATA.
029.xml                 ']]>' in PCDATA.
030.xml                 Illegal XML character (formfeed).
031.xml                 Illegal XML character (formfeed).
032.xml                 Illegal XML character (formfeed).
033.xml                 Illegal XML character (ESC).
034.xml                 Illegal XML character (formfeed).
038.xml                 Duplicate attribute assignment.
069.xml                 Missing space before 'NDATA'.
070.xml                 '--' in comment.
074.xml                 Element starting and ending in different entity.
086.xml                 '[' in public identifier.
087.xml                 '[' in public identifier.
088.xml                 ' ' in system literal.
089.xml                 NDATA parameter entity.
090.xml                 '<' in attribute value.
091.xml                 NDATA parameter entity.
096.xml                 Missing space before 'encoding'.
100.xml                 Wrong case for standalone value.
104.xml                 Element begins and ends in different entities.
116.xml                 Reference begins and ends in different entities.
117.xml                 Reference begins and ends in different entities.
119.xml                 Reference begins and ends in different entities.
140.xml                 Illegal character at start of NAME.
141.xml                 Illegal character in NAME.

My experiments suggest that reporting all of these errors would add
10-15K to AElfred's size and slow down parsing by about 15 per cent.


ABOUT THE NAME "AElfred"
-----------------------

AElfred the Great (AElfred in ASCII) was king of Wessex, and at least
nominally of all England, at the time of his death in 899AD.  AElfred
introduced a wide-spread literacy program in the hope that his people
would learn to read English, at least, if Latin was too difficult for
them.  This AElfred hopes to bring another sort of literacy to Java,
using XML, at least, if full SGML is too difficult.

The initial "AE" ("Æ" in ISO-8859-1) is also a reminder that XML is
not limited to ASCII.

__end of README__
