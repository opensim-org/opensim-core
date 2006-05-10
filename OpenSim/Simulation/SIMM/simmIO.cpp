// simmIO.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "simmIO.h"
#include <OpenSim/Tools/rdMath.h>

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

/* Read a non-comment string from a stream. Returns false if could not read
 * a string (EOF reached).
 */
bool readNonCommentString(istream &stream, string &buffer)
{
   int start, end;

	buffer = "\0";
	while (1)
	{
		if (buffer.empty())
			if (!readString(stream, buffer))
				return false;

		start = buffer.find("/*");
		if (start != buffer.npos)
		{
			while (1)
			{
				end = buffer.find("*/");
				if (end == buffer.npos) // if end was not found
					readString(stream, buffer);
				else
				{
					buffer.erase(start, end+2);
					break;
				}
			}
		}
		else
			break;
	}
	return true;
}

/* Read a string from a stream. Returns true if a string was read,
 * false if EOF was reached.
 */
bool readString(istream &stream, string &buffer)
{
   while (1)
   {
      stream >> buffer;
      if (stream.eof())
         return false;
      if (!buffer.empty())
         return true;
   }
}

/* Read a string from a line, return the string, and the line
 * with the string removed.
 */
bool readStringFromLine(string &line, string &buffer)
{
   int i;

   if (line.empty())
      return false;

   /* remove any initial whitespace */
   i = findFirstNonWhiteSpace(line);
   if (i > 0)
      line.erase(0, i);

   if (line.empty())
      return false;
   /* find first whitespace */
   i = findFirstWhiteSpace(line);

   /* only one string found */
   if (i == -1)
   {
      i = line.length();
      buffer = line;
      line.erase(0, i);
   }
   else if (i >= 0)
   {
      buffer.assign(line, 0, i);
      line.erase(0, i + 1); //remove string and whitespace
   }
   /* remove any whitespace after the string*/
   i = findFirstNonWhiteSpace(line);
   if (i > 0)
      line.erase(0, i);
   else if (i != 0)
      line.erase(0, line.length());

   if (buffer.empty())
      return false;
   return true;
}

/* Read a tab-delimited string from a line, return the string, and
 * the line with the string removed. This function strips all leading
 * white space from the line, then looks for a tab, line feed, or
 * carriage return to terminate the string.
 */
bool readTabDelimitedStringFromLine(string &line, string &buffer)
{
   int i;

   if (line.empty())
      return false;

   /* remove any initial whitespace */
   i = findFirstNonWhiteSpace(line);
   if (i > 0)
      line.erase(0, i);

   if (line.empty())
      return false;

   /* find first tab, line feed, or carriage return */
   i = line.find_first_of("\t\r\n", 0);

   /* no tab, whole line is one string */
   if (i == -1)
   {
      i = line.length();
      buffer = line;
      line.erase(0, i);
   }
   else if (i >= 0)
   {
      buffer.assign(line, 0, i);
      line.erase(0, i + 1); //remove string and whitespace
   }
   /* remove any whitespace after the string */
   i = findFirstNonWhiteSpace(line);
   if (i > 0)
      line.erase(0, i);
   else if (i != 0)
      line.erase(0, line.length());

   if (buffer.empty())
      return false;

   return true;
}

/* Read an integer from a string, return the integer and the line with
 * the integer removed.
 */
bool readIntegerFromString(string &line, int *x)
{
   int i, end;
   string buffer;

   if (line.empty())
      return false;

   /* remove any characters before the number */
   i = line.find_first_of("0123456789-", 0);
   if (i != 0)
      line.erase(0, i);

   /* remove number from line, copy number to buffer */
   i = line.find_first_not_of("0123456789-eE", 0);
   end = line.length();
   if (i != line.npos)
   {
      buffer.assign(line, 0, i);
      line.erase(0, i);
   }
   else
   {
      buffer.assign(line);
      line.erase(0, end);
   }

   /* remove any whitespace after the string*/
   i = findFirstNonWhiteSpace(line);
   if (i > 0)
      line.erase(0, i);
   
   if (buffer.empty())
      return false;
   *x = atoi(buffer.c_str());
   return true;
}


/* Read a double from a string, return the double and the line with
 * the double removed.
 */
bool readDoubleFromString(string &line, double *x)
{
   int i, end;
   string buffer;

   if (line.empty())
      return false;

   /* remove any characters before the number */
   i = line.find_first_of("0123456789-.", 0);
   if (i != 0)
      line.erase(0, i);

   /* remove number from line, copy number to buffer */
   i = line.find_first_not_of("0123456789-.eE", 0);
   end = line.length();
   if (i != line.npos)
   {
      buffer.assign(line, 0, i);
      line.erase(0, i);
   }
   //if number is at end of line
   else
   {
      buffer.assign(line);
      line.erase(0, end);
   }
   /* remove any whitespace after the string, but don't remove any tabs */
   i = findFirstNonWhiteSpace(line);
   if ((i > 0) && (line[i-1] != '\t'))
      line.erase(0, i);
   

   if (buffer.empty())
      return false;
   *x = atof(buffer.c_str());
   return true;
}

bool readVectorFromString(string &line, SimmPoint &vec)
{
   bool ok = true;

	double* vecPtr = vec.get();
   ok = ok && readDoubleFromString(line, &vecPtr[0]);
   ok = ok && readDoubleFromString(line, &vecPtr[1]);
   ok = ok && readDoubleFromString(line, &vecPtr[2]);
   return ok;
}

bool readVectorFromString(string &line, double *v1, double *v2, double *v3)
{
   bool ok = true;

   ok = ok && readDoubleFromString(line, v1);
   ok = ok && readDoubleFromString(line, v2);
   ok = ok && readDoubleFromString(line, v3);
   return ok;
}

/* READ COORDINATES: Reads TAB separated coordinate x, y, z values.
 * IF 2 TABS are found in a row, assume that there should have been data
 * in the place of the second tab and that it is missing - fill it in
 * using with rdMath::NAN.  Return when x, y, and z are set.  Return EOF
 * at the end of the line. */
bool readCoordinatesFromLine(string &line, double vec[3])
{
   int numTabs = 0, numCoords = 0;
   double value;

   while (!line.empty())
   {
      if (line[0] == '\t')
      {
         numTabs++;
         line.erase(0, 1);
      }
      else
      {
         if (!readDoubleFromString(line, &value))
         {
            return false;
         }
         else
         {
            vec[numCoords++] = value;
            numTabs = 0;
         }         
      }
      /* if you have 3 TABS in a row, coordinate data is missing */
      if (numTabs == 3)
      {
			vec[0] = vec[1] = vec[2] = rdMath::NAN;
         numCoords = 3;
      }
      if (numCoords == 3)
         break;
   }
   if (numCoords == 3)
      return true;
   else
      return false;
}

int findFirstNonWhiteSpace(string &buffer)
{
   return buffer.find_first_not_of(" \t\r\n", 0);
}

int findFirstWhiteSpace(string &buffer)
{
   return buffer.find_first_of(" \t\r\n", 0);
}
