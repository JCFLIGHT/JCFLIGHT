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

#include "MATHSUPPORT.h"

static void Println_Sizes(void)
{
    Serial.println("Tamanho das variaveis:");

    Serial.print("char      : ");
    Serial.println(sizeof(char));

    Serial.print("short     : ");
    Serial.println(sizeof(short));

    Serial.print("int       : ");
    Serial.println(sizeof(int));

    Serial.print("long      : ");
    Serial.println(sizeof(long));

    Serial.print("long long : ");
    Serial.println(sizeof(long long));

    Serial.print("bool      : ");
    Serial.println(sizeof(bool));

    Serial.print("void*     : ");
    Serial.println(sizeof(void *));

    Serial.print("Teste de NaN: ");
    Serial.println(sqrt(-1.0f));

    Serial.print("Teste de +Inf: ");
    Serial.println(1.0f / 0.0f);

    Serial.print("Teste de -Inf: ");
    Serial.println(-1.0f / 0.0f);
}

#define TENTIMES(x) \
    do              \
    {               \
        x;          \
        x;          \
        x;          \
        x;          \
        x;          \
        x;          \
        x;          \
        x;          \
        x;          \
        x;          \
    } while (0)

#define FIFTYTIMES(x) \
    do                \
    {                 \
        TENTIMES(x);  \
        TENTIMES(x);  \
        TENTIMES(x);  \
        TENTIMES(x);  \
        TENTIMES(x);  \
    } while (0)

#define TIMEIT(name, op, count)                                   \
    do                                                            \
    {                                                             \
        uint32_t us_end, us_start;                                \
        us_start = micros();                                      \
        for (uint8_t i = 0; i < count; i++)                       \
        {                                                         \
            FIFTYTIMES(op);                                       \
        }                                                         \
        us_end = micros();                                        \
        Serial.print(name);                                       \
        Serial.print("          ");                               \
        Serial.print(double(us_end - us_start) / (count * 50.0)); \
        Serial.println(" MicroSegundo/Chamada");                  \
    } while (0)

volatile float v_f = 1.0;
volatile float v_out;
volatile double v_d = 1.0;
volatile double v_out_d;
volatile uint32_t v_32 = 1;
volatile uint32_t v_out_32 = 1;
volatile uint16_t v_16 = 1;
volatile uint16_t v_out_16 = 1;
volatile uint8_t v_8 = 1;
volatile uint8_t v_out_8 = 1;
volatile uint8_t mbuf1[128], mbuf2[128];
volatile uint64_t v_64 = 1;
volatile uint64_t v_out_64 = 1;

static void Println_Timings(void)
{

    v_f = 1 + (micros() % 5);
    v_out = 1 + (micros() % 3);

    v_32 = 1 + (micros() % 5);
    v_out_32 = 1 + (micros() % 3);

    v_16 = 1 + (micros() % 5);
    v_out_16 = 1 + (micros() % 3);

    v_8 = 1 + (micros() % 5);
    v_out_8 = 1 + (micros() % 3);

    Serial.println("Tempo de operacoes:");

    TIMEIT("nop", asm volatile("nop" ::), 255);

    TIMEIT("micros()", micros(), 200);
    TIMEIT("millis()", millis(), 200);

    TIMEIT("fadd", v_out += v_f, 100);
    TIMEIT("fsub", v_out -= v_f, 100);
    TIMEIT("fmul", v_out *= v_f, 100);
    TIMEIT("fdiv /=", v_out /= v_f, 100);
    TIMEIT("fdiv 2/x", v_out = 2.0f / v_f, 100);

    TIMEIT("dadd", v_out_d += v_d, 100);
    TIMEIT("dsub", v_out_d -= v_d, 100);
    TIMEIT("dmul", v_out_d *= v_d, 100);
    TIMEIT("ddiv", v_out_d /= v_d, 100);

    TIMEIT("sin()", v_out = Fast_Sine(v_f), 20);
    TIMEIT("cos()", v_out = Fast_Cosine(v_f), 20);
    TIMEIT("tan()", v_out = Fast_Tangent(v_f), 20);
    TIMEIT("acos()", v_out = Fast_AtanCosine(v_f * 0.2), 20);
    TIMEIT("asin()", v_out = asinf(v_f * 0.2), 20);
    TIMEIT("fast_atan2()", v_out = Fast_Atan2(v_f * 0.2, v_f * 0.3), 20);
    TIMEIT("sqrt()", v_out = Fast_SquareRoot(v_f), 20);

    TIMEIT("iadd8", v_out_8 += v_8, 100);
    TIMEIT("isub8", v_out_8 -= v_8, 100);
    TIMEIT("imul8", v_out_8 *= v_8, 100);
    TIMEIT("idiv8", v_out_8 /= v_8, 100);

    TIMEIT("iadd16", v_out_16 += v_16, 100);
    TIMEIT("isub16", v_out_16 -= v_16, 100);
    TIMEIT("imul16", v_out_16 *= v_16, 100);
    TIMEIT("idiv16", v_out_16 /= v_16, 100);

    TIMEIT("iadd32", v_out_32 += v_32, 100);
    TIMEIT("isub32", v_out_32 -= v_32, 100);
    TIMEIT("imul32", v_out_32 *= v_32, 100);
    TIMEIT("idiv32", v_out_32 /= v_32, 100);

    TIMEIT("iadd64", v_out_64 += v_64, 100);
    TIMEIT("isub64", v_out_64 -= v_64, 100);
    TIMEIT("imul64", v_out_64 *= v_64, 100);
    TIMEIT("idiv64", v_out_64 /= v_64, 100);

    TIMEIT("memcpy128", memcpy((void *)mbuf1, (const void *)mbuf2, sizeof(mbuf1)), 20);
    TIMEIT("memset128", memset((void *)mbuf1, 1, sizeof(mbuf1)), 20);
    TIMEIT("delay(1)", delay(1), 5);
}

void setup()
{
    Serial.begin(115200);
    Println_Sizes();
    Serial.println("");
    Println_Timings();
}

void loop()
{
}
