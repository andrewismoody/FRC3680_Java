package frc.robot.auto;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.fasterxml.jackson.databind.JsonNode;

public final class AutoExpr {
    private AutoExpr() {}

    public static Double eval(
            String expr,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver) {

        return eval(expr, scalarResolver, vec2Resolver, null);
    }

    public static Double eval(
            String expr,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver,
            BiFunction<String, Character, Double> vec3Resolver) {

        if (expr == null) return null;
        expr = expr.trim();
        if (expr.isEmpty()) return null;

        try {
            Parser p = new Parser(expr, scalarResolver, vec2Resolver, vec3Resolver);
            Double v = p.parseExpression();
            p.skipWs();
            if (!p.eof()) return null; // trailing junk
            return v;
        } catch (RuntimeException ex) {
            return null;
        }
    }

    public static Double evalNode(
            JsonNode node,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver) {

        return evalNode(node, scalarResolver, vec2Resolver, null);
    }

    public static Double evalNode(
            JsonNode node,
            Function<String, Double> scalarResolver,
            BiFunction<String, Character, Double> vec2Resolver,
            BiFunction<String, Character, Double> vec3Resolver) {

        if (node == null || node.isNull()) return null;
        if (node.isNumber()) return node.doubleValue();
        if (node.isTextual()) return eval(node.asText(), scalarResolver, vec2Resolver, vec3Resolver);
        return null;
    }

    private static final class Parser {
        private final String s;
        private final int n;
        private int i = 0;

        private final Function<String, Double> scalarResolver;
        private final BiFunction<String, Character, Double> vec2Resolver;
        private final BiFunction<String, Character, Double> vec3Resolver;

        Parser(String s,
               Function<String, Double> scalarResolver,
               BiFunction<String, Character, Double> vec2Resolver,
               BiFunction<String, Character, Double> vec3Resolver) {
            this.s = s;
            this.n = s.length();
            this.scalarResolver = scalarResolver;
            this.vec2Resolver = vec2Resolver;
            this.vec3Resolver = vec3Resolver;
        }

        boolean eof() { return i >= n; }

        void skipWs() {
            while (i < n) {
                char c = s.charAt(i);
                if (c == ' ' || c == '\t' || c == '\r' || c == '\n') i++;
                else break;
            }
        }

        boolean eat(char c) {
            skipWs();
            if (i < n && s.charAt(i) == c) {
                i++;
                return true;
            }
            return false;
        }

        char peek() {
            skipWs();
            return (i < n) ? s.charAt(i) : '\0';
        }

        // expression := term (('+'|'-') term)*
        Double parseExpression() {
            Double v = parseTerm();
            if (v == null) return null;

            while (true) {
                if (eat('+')) {
                    Double r = parseTerm();
                    if (r == null) return null;
                    v = v + r;
                } else if (eat('-')) {
                    Double r = parseTerm();
                    if (r == null) return null;
                    v = v - r;
                } else {
                    break;
                }
            }
            return v;
        }

        // term := unary (('*'|'/') unary)*
        Double parseTerm() {
            Double v = parseUnary();
            if (v == null) return null;

            while (true) {
                if (eat('*')) {
                    Double r = parseUnary();
                    if (r == null) return null;
                    v = v * r;
                } else if (eat('/')) {
                    Double r = parseUnary();
                    if (r == null) return null;
                    v = v / r;
                } else {
                    break;
                }
            }
            return v;
        }

        // unary := ('+'|'-') unary | primary
        Double parseUnary() {
            if (eat('+')) return parseUnary();
            if (eat('-')) {
                Double v = parseUnary();
                return (v == null) ? null : -v;
            }
            return parsePrimary();
        }

        // primary := number | var | '(' expression ')'
        Double parsePrimary() {
            skipWs();

            if (eat('(')) {
                Double v = parseExpression();
                if (v == null) return null;
                if (!eat(')')) return null;
                return v;
            }

            char c = peek();
            if (c == '$') return parseVar();
            if (isNumberStart(c)) return parseNumber();

            return null;
        }

        Double parseVar() {
            if (!eat('$')) return null;

            skipWs();
            int start = i;

            // accept [A-Za-z0-9_]
            while (i < n) {
                char c = s.charAt(i);
                if (Character.isLetterOrDigit(c) || c == '_') i++;
                else break;
            }

            if (i == start) return null;
            String name = s.substring(start, i);

            skipWs();
            if (eat('.')) {
                skipWs();
                if (i >= n) return null;
                char comp = Character.toUpperCase(s.charAt(i));
                if (comp != 'X' && comp != 'Y' && comp != 'Z') return null;
                i++;

                if (vec3Resolver != null) {
                    Double v3 = vec3Resolver.apply(name, comp);
                    if (v3 != null) return v3;
                }
                if (vec2Resolver != null) {
                    return vec2Resolver.apply(name, comp);
                }
                return null;
            }

            return (scalarResolver != null) ? scalarResolver.apply(name) : null;
        }

        Double parseNumber() {
            skipWs();
            int start = i;

            // digits/decimal/exponent
            boolean sawDot = false;
            while (i < n) {
                char c = s.charAt(i);
                if (Character.isDigit(c)) {
                    i++;
                } else if (c == '.' && !sawDot) {
                    sawDot = true;
                    i++;
                } else {
                    break;
                }
            }

            // optional exponent
            if (i < n && (s.charAt(i) == 'e' || s.charAt(i) == 'E')) {
                int epos = i;
                i++;
                if (i < n && (s.charAt(i) == '+' || s.charAt(i) == '-')) i++;
                int expStart = i;
                while (i < n && Character.isDigit(s.charAt(i))) i++;
                if (i == expStart) i = epos; // roll back if invalid exponent
            }

            if (i == start) return null;

            try {
                return Double.valueOf(s.substring(start, i));
            } catch (NumberFormatException ex) {
                return null;
            }
        }

        static boolean isNumberStart(char c) {
            return Character.isDigit(c) || c == '.';
        }
    }
}
