/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#ifndef STRINGSUPPORT_H_
#define STRINGSUPPORT_H_

//COPIADO DA APPLE.INC

int inline ToLower(int chr)
{
    return (chr >= 'A' && chr <= 'Z') ? (chr + 32) : (chr);
}

int inline StringCompare(const char *s1, const char *s2, size_t n)
{
    if (n == 0)
    {
        return 0;
    }
    while (n-- != 0 && ToLower(*s1) == ToLower(*s2))
    {
        if (n == 0 || *s1 == '\0' || *s2 == '\0')
        {
            break;
        }
        s1++;
        s2++;
    }
    return ToLower(*(const unsigned char *)s1) - ToLower(*(const unsigned char *)s2);
}
#endif