import math

class Term:
    def __init__(self, var, coeff=1, power=1):
        if power == 0:
            self.var = 0
        else:
            self.var = var
        self.coeff = coeff
        self.power = power
    
    def from_term(cls, other):
        return cls(other.var, other.coeff, other.power)

    def derivative(self):
        self.coeff *= self.power
        self.power -= 1

    def format_term(self):
        return str(self)

    def __str__(self):
        if self.power == 0:
            return str(self.coeff)

        coeff_part = ''
        if self.coeff == -1:
            coeff_part = '-'
        elif self.coeff != 1:
            coeff_part = str(self.coeff)
            
        var_part = self.var
        power_part = f'^{self.power}' if self.power != 1 else ''
        
        return f"{coeff_part}{var_part}{power_part}"

    __repr__ = __str__

def partial_derivative(expression, var):
    out = []
    for term in expression:
        if term.var == var:
            new_term = Term(term.var, term.coeff, term.power)
            new_term.derivative()
            out.append(new_term)
    return out

def format_expression(terms):
    if not terms:
        return "0"
    formatted = []
    for term in terms:
        term_str = term.format_term()
        if term_str.startswith('-'):
            formatted.append(term_str)
        else:
            formatted.append('+' + term_str)
    result = ' '.join(formatted)
    return result[1:] if result.startswith('+') else result

def create_gradient(expression, vars):
    out = []
    for var in vars:
        derivative = partial_derivative(expression.copy(), var)
        out.append(f"∂/∂{var} = {format_expression(derivative)}")
    return '\n'.join(out)

def main():
    print("Math Practice App")
    exp = [Term("x", 2, 3), Term("x", 3, 2), Term("y", -7, 2), Term('y', 5, 1)]
    print(f"Original expression: {format_expression(exp)}")
    print("\nGradient:")
    print(create_gradient(exp, ['x', 'y']))
    print("\nThe gradient vector points to where the most rate of change will be discovered!")


if __name__ == "__main__":
    main()